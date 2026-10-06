"""Simulation entry points for CLI, API, and regression tests."""

from ._main_models import FullMissionResult, SimulationLog
from ._run_full_mission import run_full_mission
from ._run_simulation import run_simulation
from ._simulation_step import check_termination, simulation_step

__all__ = [
    "FullMissionResult",
    "SimulationLog",
    "check_termination",
    "run_full_mission",
    "run_simulation",
    "simulation_step",
]


if __name__ == "__main__":
    run_simulation()