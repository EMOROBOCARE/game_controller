"""Unit tests for auto-advance scheduling."""

import game_controller.auto_advance as auto_advance


def test_schedule_and_fire(monkeypatch):
    timers = []

    class RecordingTimer:
        def __init__(self, interval, function, args=None, kwargs=None):
            self.interval = interval
            self.function = function
            self.args = args or ()
            self.kwargs = kwargs or {}
            self.started = False
            self.cancelled = False
            timers.append(self)

        def start(self):
            self.started = True

        def cancel(self):
            self.cancelled = True

    monkeypatch.setattr(auto_advance.threading, "Timer", RecordingTimer)

    fired = []
    config = auto_advance.AutoAdvanceConfig(phase_intro=0.1)
    scheduler = auto_advance.AutoAdvanceScheduler(config, lambda tx: fired.append(tx))

    assert scheduler.schedule_if_needed("PHASE_INTRO", 1) is True
    assert len(timers) == 1
    assert timers[0].interval == 0.1

    timers[0].function(*timers[0].args, **timers[0].kwargs)
    assert fired == [1]

    # Already completed; no re-schedule
    assert scheduler.schedule_if_needed("PHASE_INTRO", 1) is False


def test_mark_completed_prevents_callback(monkeypatch):
    timers = []

    class RecordingTimer:
        def __init__(self, interval, function, args=None, kwargs=None):
            self.interval = interval
            self.function = function
            self.args = args or ()
            self.kwargs = kwargs or {}
            self.started = False
            self.cancelled = False
            timers.append(self)

        def start(self):
            self.started = True

        def cancel(self):
            self.cancelled = True

    monkeypatch.setattr(auto_advance.threading, "Timer", RecordingTimer)

    fired = []
    scheduler = auto_advance.AutoAdvanceScheduler(
        auto_advance.AutoAdvanceConfig(phase_intro=0.1),
        lambda tx: fired.append(tx),
    )

    scheduler.schedule_if_needed("PHASE_INTRO", 5)
    scheduler.mark_completed(5)
    timers[0].function(*timers[0].args, **timers[0].kwargs)
    assert fired == []


def test_cancel_timer(monkeypatch):
    timers = []

    class RecordingTimer:
        def __init__(self, interval, function, args=None, kwargs=None):
            self.interval = interval
            self.function = function
            self.args = args or ()
            self.kwargs = kwargs or {}
            self.started = False
            self.cancelled = False
            timers.append(self)

        def start(self):
            self.started = True

        def cancel(self):
            self.cancelled = True

    monkeypatch.setattr(auto_advance.threading, "Timer", RecordingTimer)

    scheduler = auto_advance.AutoAdvanceScheduler(
        auto_advance.AutoAdvanceConfig(phase_intro=0.1),
        lambda tx: None,
    )

    scheduler.schedule_if_needed("PHASE_INTRO", 2)
    assert scheduler.cancel(2) is True
    assert timers[0].cancelled is True


def test_no_schedule_for_wait_input():
    scheduler = auto_advance.AutoAdvanceScheduler(
        auto_advance.AutoAdvanceConfig(),
        lambda tx: None,
    )
    assert scheduler.schedule_if_needed("WAIT_INPUT", 1) is False
