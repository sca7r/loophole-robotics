"""Robot lifecycle FSM — the PRD's "Behavior Tree / FSM" layer, v1 shape.

Per the design discussion we deliberately chose a **finite state machine, not
a behavior tree**, for version 1: pick-and-place programs are linear sequences
with failure handling, which a 5-state FSM models completely. Behavior trees
earn their complexity only with conditional, hierarchical behaviors (vision,
recovery strategies, tool selection) — none of which are in v1 scope. The
Skill Engine's interfaces are BT-compatible, so a BT can be layered on later
without redesign.

States::

    IDLE ──connect──▶ READY ──start──▶ EXECUTING
                        ▲                 │
                        └──────done───────┘
                        │                 │
                        │               fault
                      reset               ▼
                        └───────────── ERROR

    any state ──shutdown──▶ SHUTDOWN  (terminal)

Transitions are validated: an invalid event for the current state raises
:class:`InvalidTransitionError` rather than silently corrupting the lifecycle.
"""
from __future__ import annotations

import enum
import logging
from dataclasses import dataclass, field

logger = logging.getLogger("loophole_arm.lifecycle")


class State(enum.Enum):
    IDLE = "idle"            # constructed, not connected
    READY = "ready"          # connected + enabled, awaiting work
    EXECUTING = "executing"  # running a skill / sequence
    ERROR = "error"          # fault; requires reset
    SHUTDOWN = "shutdown"    # terminal


class Event(enum.Enum):
    CONNECT = "connect"
    START = "start"
    DONE = "done"
    FAULT = "fault"
    RESET = "reset"
    SHUTDOWN = "shutdown"


class InvalidTransitionError(RuntimeError):
    """Raised when an event is not legal in the current state."""


# (state, event) -> new state. Anything absent is invalid.
_TRANSITIONS: dict[tuple[State, Event], State] = {
    (State.IDLE, Event.CONNECT): State.READY,
    (State.READY, Event.START): State.EXECUTING,
    (State.EXECUTING, Event.DONE): State.READY,
    (State.EXECUTING, Event.FAULT): State.ERROR,
    (State.READY, Event.FAULT): State.ERROR,
    (State.ERROR, Event.RESET): State.READY,
    # SHUTDOWN is reachable from every non-terminal state.
    (State.IDLE, Event.SHUTDOWN): State.SHUTDOWN,
    (State.READY, Event.SHUTDOWN): State.SHUTDOWN,
    (State.EXECUTING, Event.SHUTDOWN): State.SHUTDOWN,
    (State.ERROR, Event.SHUTDOWN): State.SHUTDOWN,
}


@dataclass
class Lifecycle:
    """The robot's lifecycle state machine.

    Owns nothing but the state; the :class:`RobotController` (or server)
    drives it and consults :meth:`can` before acting.
    """
    state: State = State.IDLE
    history: list[tuple[State, Event, State]] = field(default_factory=list, repr=False)

    def transition(self, event: Event) -> State:
        """Apply ``event``; return the new state or raise :class:`InvalidTransitionError`."""
        key = (self.state, event)
        if key not in _TRANSITIONS:
            raise InvalidTransitionError(
                f"event {event.value!r} not valid in state {self.state.value!r}"
            )
        new = _TRANSITIONS[key]
        self.history.append((self.state, event, new))
        logger.info("lifecycle: %s --%s--> %s", self.state.value, event.value, new.value)
        self.state = new
        return new

    def can(self, event: Event) -> bool:
        """True if ``event`` is legal in the current state."""
        return (self.state, event) in _TRANSITIONS

    @property
    def is_terminal(self) -> bool:
        return self.state == State.SHUTDOWN
