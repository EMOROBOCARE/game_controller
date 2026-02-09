"""Auto-advance scheduling for non-interactive state transitions.

This module handles scheduling ON_COMPLETE events for states that
require automatic progression (e.g., PHASE_INTRO, CORRECT, FAIL_L1/L2).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Set


@dataclass
class AutoAdvanceConfig:
    """Configuration for auto-advance timeouts per game state."""
    
    phase_intro: float = 2.0
    round_setup: float = 0.05
    question_present: float = 0.05
    fail_l1: float = 2.0
    fail_l2: float = 2.0
    correct: float = 0.6
    phase_complete: float = 0.3
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AutoAdvanceConfig":
        """Create config from dictionary (e.g., loaded from YAML)."""
        return cls(
            phase_intro=float(data.get("phase_intro", 2.0)),
            round_setup=float(data.get("round_setup", 0.05)),
            question_present=float(data.get("question_present", 0.05)),
            fail_l1=float(data.get("fail_l1", 2.0)),
            fail_l2=float(data.get("fail_l2", 2.0)),
            correct=float(data.get("correct", 0.6)),
            phase_complete=float(data.get("phase_complete", 0.3)),
        )
    
    def get_timeout_for_state(self, game_state: str) -> Optional[float]:
        """Get auto-advance timeout for a given game state.
        
        Returns None for states that should not auto-advance (e.g., WAIT_INPUT).
        """
        state_upper = (game_state or "").upper()
        
        timeout_map = {
            "PHASE_INTRO": self.phase_intro,
            "ROUND_SETUP": self.round_setup,
            "QUESTION_PRESENT": self.question_present,
            "FAIL_L1": self.fail_l1,
            "FAIL_L2": self.fail_l2,
            "CORRECT": self.correct,
            "PHASE_COMPLETE": self.phase_complete,
        }
        
        return timeout_map.get(state_upper)


class AutoAdvanceScheduler:
    """Manages scheduling of ON_COMPLETE events for auto-advancing states.
    
    Thread-safe scheduler that:
    - Schedules ON_COMPLETE callbacks after configured timeouts
    - Tracks scheduled transaction IDs to prevent duplicates
    - Supports cancellation when state changes externally
    """
    
    def __init__(
        self,
        config: AutoAdvanceConfig,
        on_complete_callback: Callable[[int], None],
        logger: Optional[Any] = None,
    ) -> None:
        """Initialize the scheduler.
        
        Args:
            config: Auto-advance timeout configuration
            on_complete_callback: Function to call with transaction_id when timer fires
            logger: Optional logger for debug output
        """
        self._config = config
        self._on_complete_callback = on_complete_callback
        self._logger = logger
        
        self._lock = threading.Lock()
        self._pending_timers: Dict[int, threading.Timer] = {}
        self._completed_transactions: Set[int] = set()
    
    def schedule_if_needed(
        self,
        game_state: str,
        transaction_id: int,
    ) -> bool:
        """Schedule an ON_COMPLETE for the given state if auto-advance is needed.
        
        Args:
            game_state: Current game state (e.g., "PHASE_INTRO")
            transaction_id: Transaction ID from decision_state
            
        Returns:
            True if a timer was scheduled, False otherwise
        """
        timeout = self._config.get_timeout_for_state(game_state)
        
        if timeout is None:
            if self._logger:
                self._logger.debug(
                    f"No auto-advance for state {game_state}"
                )
            return False
        
        with self._lock:
            # Don't schedule if already pending or completed
            if transaction_id in self._pending_timers:
                if self._logger:
                    self._logger.debug(
                        f"Transaction {transaction_id} already scheduled"
                    )
                return False
                
            if transaction_id in self._completed_transactions:
                if self._logger:
                    self._logger.debug(
                        f"Transaction {transaction_id} already completed"
                    )
                return False
            
            # Schedule the timer
            timer = threading.Timer(
                timeout,
                self._timer_callback,
                args=(transaction_id,),
            )
            timer.daemon = True
            self._pending_timers[transaction_id] = timer
            timer.start()
            
            if self._logger:
                self._logger.info(
                    f"Scheduled ON_COMPLETE for {game_state} "
                    f"(tx={transaction_id}) in {timeout}s"
                )
            
            return True
    
    def cancel(self, transaction_id: int) -> bool:
        """Cancel a pending auto-advance timer.
        
        Args:
            transaction_id: Transaction ID to cancel
            
        Returns:
            True if a timer was cancelled, False if not found
        """
        with self._lock:
            timer = self._pending_timers.pop(transaction_id, None)
            if timer:
                timer.cancel()
                if self._logger:
                    self._logger.debug(
                        f"Cancelled auto-advance for tx={transaction_id}"
                    )
                return True
            return False
    
    def cancel_all(self) -> int:
        """Cancel all pending timers.
        
        Returns:
            Number of timers cancelled
        """
        with self._lock:
            count = 0
            for tx_id, timer in list(self._pending_timers.items()):
                timer.cancel()
                count += 1
            self._pending_timers.clear()
            if self._logger and count > 0:
                self._logger.debug(f"Cancelled {count} pending auto-advance timers")
            return count
    
    def mark_completed(self, transaction_id: int) -> None:
        """Mark a transaction as completed externally (e.g., user input).
        
        This prevents the auto-advance from firing for this transaction.
        """
        with self._lock:
            self._completed_transactions.add(transaction_id)
            # Also cancel any pending timer
            timer = self._pending_timers.pop(transaction_id, None)
            if timer:
                timer.cancel()
        
        # Cleanup old completed transactions (keep last 100)
        if len(self._completed_transactions) > 100:
            with self._lock:
                # Remove oldest entries
                excess = len(self._completed_transactions) - 100
                sorted_txs = sorted(self._completed_transactions)
                for tx in sorted_txs[:excess]:
                    self._completed_transactions.discard(tx)
    
    def _timer_callback(self, transaction_id: int) -> None:
        """Internal callback when timer fires."""
        with self._lock:
            # Remove from pending
            self._pending_timers.pop(transaction_id, None)
            
            # Check if already completed externally
            if transaction_id in self._completed_transactions:
                if self._logger:
                    self._logger.debug(
                        f"Auto-advance skipped for tx={transaction_id} "
                        "(already completed)"
                    )
                return
            
            # Mark as completed
            self._completed_transactions.add(transaction_id)
        
        # Call the callback outside the lock
        if self._logger:
            self._logger.info(
                f"Auto-advance firing ON_COMPLETE for tx={transaction_id}"
            )
        
        try:
            self._on_complete_callback(transaction_id)
        except Exception as e:
            if self._logger:
                self._logger.error(
                    f"Error in auto-advance callback: {e}"
                )
