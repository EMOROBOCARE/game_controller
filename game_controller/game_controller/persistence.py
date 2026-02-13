"""PostgreSQL persistence helpers for users and interaction logs."""

from __future__ import annotations

import os
import threading
import uuid
from typing import Any, Dict, List, Optional, Tuple

try:
    import psycopg2
except Exception:  # pragma: no cover - dependency can be absent in some envs
    psycopg2 = None  # type: ignore[assignment]


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class GameControllerPersistence:
    """Thin DB adapter for child list and answer logs."""

    def __init__(self, logger: Any) -> None:
        self._logger = logger
        self._lock = threading.Lock()
        self._conn = None
        self._child_phase_hint_table_ready = False
        self._db_enabled = _as_bool(os.environ.get("GAME_CONTROLLER_DB_ENABLED"), True)
        self._db_cfg = {
            "dbname": os.environ.get("POSTGRES_DB", "emorobcare_db"),
            "user": os.environ.get("POSTGRES_USER", "emorobcare"),
            "password": os.environ.get("POSTGRES_PASSWORD", "D3B3rdad.Emylio"),
            "host": os.environ.get("POSTGRES_HOST", "10.147.19.11"),
            "port": int(os.environ.get("POSTGRES_PORT", "5432")),
            "connect_timeout": int(os.environ.get("POSTGRES_CONNECT_TIMEOUT_SEC", "3")),
        }

    @property
    def enabled(self) -> bool:
        return bool(self._db_enabled and psycopg2 is not None)

    def _get_conn(self):
        if not self.enabled:
            return None
        with self._lock:
            try:
                if self._conn is not None and getattr(self._conn, "closed", 1) == 0:
                    return self._conn
            except Exception:
                self._conn = None
            try:
                self._conn = psycopg2.connect(**self._db_cfg)
                self._conn.autocommit = True
                self._logger.info(
                    "[GC][DB] Connected to "
                    f"{self._db_cfg['host']}:{self._db_cfg['port']}/{self._db_cfg['dbname']}"
                )
                return self._conn
            except Exception as exc:
                self._logger.warn(f"[GC][DB] Connection failed: {exc}")
                self._conn = None
                return None

    def _ensure_child_phase_hint_table(self, conn: Any) -> bool:
        if self._child_phase_hint_table_ready:
            return True
        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    CREATE TABLE IF NOT EXISTS child_phase_hint_usage (
                        child_id TEXT NOT NULL,
                        game_slug TEXT NOT NULL,
                        phase_code TEXT NOT NULL,
                        hint_count INTEGER NOT NULL DEFAULT 0,
                        updated_at TIMESTAMP NOT NULL DEFAULT NOW(),
                        PRIMARY KEY (child_id, game_slug, phase_code)
                    )
                    """
                )
            self._child_phase_hint_table_ready = True
            return True
        except Exception as exc:
            self._logger.warn(f"[GC][DB] Failed to ensure child hint tracking table: {exc}")
            return False

    def load_children(self) -> List[Tuple[str, str]]:
        """Return list of (child_id, child_name)."""
        conn = self._get_conn()
        if conn is None:
            return []
        try:
            with conn.cursor() as cur:
                cur.execute("SELECT id, name FROM children ORDER BY name ASC, id ASC")
                rows = cur.fetchall() or []
            return [(str(row[0]), str(row[1])) for row in rows if row and len(row) >= 2]
        except Exception as exc:
            self._logger.warn(f"[GC][DB] Failed to load children: {exc}")
            return []

    def insert_interaction_log(
        self,
        *,
        session_uuid: uuid.UUID,
        child_id: Optional[str],
        game_slug: str,
        question_number: int,
        phase_code: Optional[str],
        difficulty_code: Optional[str],
        input_type: Optional[str],
        raw_input: Optional[str],
        is_correct: Optional[bool],
        classification: Optional[str],
        feedback: Optional[str] = None,
        hint_level: Optional[int] = None,
        duration_seconds: Optional[float] = None,
        failure_level: Optional[int] = None,
    ) -> bool:
        conn = self._get_conn()
        if conn is None:
            return False
        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO interaction_logs (
                        session_id,
                        child_id,
                        game_slug,
                        question_number,
                        phase_code,
                        difficulty_code,
                        failure_level,
                        input_type,
                        raw_input,
                        is_correct,
                        classification,
                        feedback,
                        hint_level,
                        duration_seconds
                    ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                    """,
                    (
                        str(session_uuid),
                        child_id,
                        str(game_slug or "unknown"),
                        int(question_number),
                        phase_code,
                        difficulty_code,
                        failure_level,
                        input_type,
                        raw_input,
                        is_correct,
                        classification,
                        feedback,
                        hint_level,
                        duration_seconds,
                    ),
                )
            return True
        except Exception as exc:
            self._logger.warn(f"[GC][DB] Failed to write interaction log: {exc}")
            return False

    def get_child_phase_hint_count(
        self,
        *,
        child_id: str,
        game_slug: str,
        phase_code: str,
    ) -> int:
        if not child_id or not game_slug or not phase_code:
            return 0
        conn = self._get_conn()
        if conn is None:
            return 0
        if not self._ensure_child_phase_hint_table(conn):
            return 0
        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT hint_count
                    FROM child_phase_hint_usage
                    WHERE child_id=%s AND game_slug=%s AND phase_code=%s
                    """,
                    (str(child_id), str(game_slug), str(phase_code).upper()),
                )
                row = cur.fetchone()
            if row and len(row) > 0:
                return int(row[0] or 0)
        except Exception as exc:
            self._logger.warn(f"[GC][DB] Failed to read child hint count: {exc}")
        return 0

    def record_child_phase_hint_usage(
        self,
        *,
        child_id: str,
        game_slug: str,
        phase_code: str,
    ) -> bool:
        if not child_id or not game_slug or not phase_code:
            return False
        conn = self._get_conn()
        if conn is None:
            return False
        if not self._ensure_child_phase_hint_table(conn):
            return False
        try:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    INSERT INTO child_phase_hint_usage (
                        child_id,
                        game_slug,
                        phase_code,
                        hint_count,
                        updated_at
                    ) VALUES (%s, %s, %s, 1, NOW())
                    ON CONFLICT (child_id, game_slug, phase_code) DO UPDATE
                    SET hint_count = child_phase_hint_usage.hint_count + 1,
                        updated_at = NOW()
                    """,
                    (str(child_id), str(game_slug), str(phase_code).upper()),
                )
            return True
        except Exception as exc:
            self._logger.warn(f"[GC][DB] Failed to record child hint usage: {exc}")
            return False
