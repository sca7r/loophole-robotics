"""FSM tests: valid transitions succeed, invalid ones raise."""
from __future__ import annotations

import pytest

from loophole_arm.control.lifecycle import (
    Event,
    InvalidTransitionError,
    Lifecycle,
    State,
)


def test_happy_path() -> None:
    lc = Lifecycle()
    assert lc.state == State.IDLE
    lc.transition(Event.CONNECT)
    assert lc.state == State.READY
    lc.transition(Event.START)
    assert lc.state == State.EXECUTING
    lc.transition(Event.DONE)
    assert lc.state == State.READY


def test_fault_and_recovery() -> None:
    lc = Lifecycle()
    lc.transition(Event.CONNECT)
    lc.transition(Event.START)
    lc.transition(Event.FAULT)
    assert lc.state == State.ERROR
    lc.transition(Event.RESET)
    assert lc.state == State.READY


def test_shutdown_from_every_state() -> None:
    for path in ([], [Event.CONNECT], [Event.CONNECT, Event.START],
                 [Event.CONNECT, Event.FAULT]):
        lc = Lifecycle()
        for e in path:
            lc.transition(e)
        lc.transition(Event.SHUTDOWN)
        assert lc.state == State.SHUTDOWN
        assert lc.is_terminal


def test_invalid_transition_raises() -> None:
    lc = Lifecycle()
    with pytest.raises(InvalidTransitionError):
        lc.transition(Event.START)          # can't start before connect
    lc.transition(Event.CONNECT)
    with pytest.raises(InvalidTransitionError):
        lc.transition(Event.CONNECT)        # can't connect twice
    lc.transition(Event.SHUTDOWN)
    with pytest.raises(InvalidTransitionError):
        lc.transition(Event.RESET)          # terminal is terminal


def test_can_checks_without_mutating() -> None:
    lc = Lifecycle()
    assert lc.can(Event.CONNECT)
    assert not lc.can(Event.START)
    assert lc.state == State.IDLE           # can() must not mutate


def test_history_records_transitions() -> None:
    lc = Lifecycle()
    lc.transition(Event.CONNECT)
    lc.transition(Event.START)
    assert lc.history == [
        (State.IDLE, Event.CONNECT, State.READY),
        (State.READY, Event.START, State.EXECUTING),
    ]
