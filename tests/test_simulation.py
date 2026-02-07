"""Tests for the simulation manager."""

import numpy as np
import pytest

from backend.physics import Params
from backend.simulation import Simulation


@pytest.fixture
def sim():
    return Simulation()


def test_initial_state(sim):
    """Simulation should start at equilibrium."""
    state = sim.step()
    assert state["type"] == "state"
    assert state["drone"]["z"] == pytest.approx(sim.params.L, abs=0.1)
    assert state["weight"]["z"] == pytest.approx(0.0, abs=0.1)


def test_step_response_convergence(sim):
    """Weight should converge to goal position within reasonable time."""
    sim.set_goal(5.0, 0.0, 0.0)

    # Run for 15 seconds (750 steps at dt=0.02)
    for _ in range(750):
        state = sim.step()

    # Weight should be close to goal
    assert state["weight"]["x"] == pytest.approx(5.0, abs=0.5)
    assert state["weight"]["z"] == pytest.approx(0.0, abs=0.5)


def test_vertical_step_response(sim):
    """Weight should track vertical goal."""
    sim.set_goal(0.0, 0.0, 3.0)

    for _ in range(500):
        state = sim.step()

    assert state["weight"]["z"] == pytest.approx(3.0, abs=0.5)


def test_multi_goal_tracking(sim):
    """System should track sequential goals."""
    # First goal
    sim.set_goal(3.0, 0.0, 0.0)
    for _ in range(500):
        state = sim.step()
    assert state["weight"]["x"] == pytest.approx(3.0, abs=0.5)

    # Second goal
    sim.set_goal(-2.0, 4.0, 2.0)
    for _ in range(750):
        state = sim.step()
    assert state["weight"]["x"] == pytest.approx(-2.0, abs=0.5)
    assert state["weight"]["y"] == pytest.approx(4.0, abs=0.5)
    assert state["weight"]["z"] == pytest.approx(2.0, abs=0.5)


def test_reset(sim):
    """Reset should return to initial state."""
    sim.set_goal(5.0, 5.0, 5.0)
    for _ in range(100):
        sim.step()

    sim.reset()
    state = sim.step()

    assert state["drone"]["x"] == pytest.approx(0.0, abs=0.1)
    assert state["drone"]["y"] == pytest.approx(0.0, abs=0.1)
    assert state["time"] == pytest.approx(0.02, abs=0.01)


def test_swing_damping(sim):
    """After reaching goal, swing angles should be small."""
    sim.set_goal(3.0, 0.0, 0.0)

    for _ in range(1000):
        state = sim.step()

    # Angles should be small (< 2 degrees)
    assert abs(state["phi_x"]) < 0.035  # ~2 degrees in radians
    assert abs(state["phi_y"]) < 0.035


def test_state_dict_keys(sim):
    """State dict should have all required keys."""
    state = sim.step()
    assert "type" in state
    assert "time" in state
    assert "drone" in state
    assert "weight" in state
    assert "phi_x" in state
    assert "phi_y" in state
    assert "goal" in state
    assert "control" in state

    assert "x" in state["drone"]
    assert "y" in state["drone"]
    assert "z" in state["drone"]
    assert "Fx" in state["control"]
