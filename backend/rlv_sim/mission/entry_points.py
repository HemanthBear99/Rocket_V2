"""Simulation entry points for CLI, API, and regression tests."""

from rlv_sim.mission.full_mission import run_full_mission
from rlv_sim.mission.models import FullMissionResult, SimulationLog
from rlv_sim.mission.single_vehicle import run_simulation
from rlv_sim.mission.step import check_termination, simulation_step

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