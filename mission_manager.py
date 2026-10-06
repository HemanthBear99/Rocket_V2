"""
RLV Mission Manager

This module handles the high-level state machine for the mission.
It defines discrete mission phases and handles the transition logic between them.

All transitions are physics-based:
  - MECO:             propellant exhaustion (m <= m_dry)
  - Stage separation: 3 s coast after MECO (pyrotechnic delay)
  - Apogee:           radial velocity sign change (r_dot: + -> -)
  - Booster flip:     attitude alignment with retrograde (dot product check)
  - Boostback:        horizontal velocity toward site cancelled
  - Entry interface:  Knudsen number / altitude threshold (70 km)
  - Landing burn:     energy-based ignition (Tsiolkovsky work integral)
"""

import logging
from enum import Enum, auto

import numpy as np

from . import constants as C
from ._mission_manager_helpers import (
    check_attitude_aligned,
    compute_downrange_distance,
    compute_horizontal_velocity,
    compute_orbit_metrics,
    compute_radial_velocity,
    compute_suicide_burn_altitude,
)
from .config_definition import SimulationConfig
from .config_factory import create_default_config
from .recovery import (
    booster_min_propellant_after_boostback,
    booster_propellant_remaining,
    compute_powered_descent_lead_time,
    estimate_ballistic_impact_to_pad,
    estimate_recovery_targeting,
    estimate_suicide_burn,
    is_near_pad_target,
    rotating_launch_site_eci,
    target_landing_site_eci,
)
from .state import State
from .utils import compute_ground_relative_velocity

logger = logging.getLogger(__name__)


class MissionPhase(Enum):
    PRELAUNCH = auto()
    ASCENT = auto()
    COAST = auto()                                                       
    STAGE_SEPARATION = auto()                                    
    S2_COAST_TO_APOGEE = auto()                          
    ORBIT_INSERTION = auto()                                                
    ORBIT_ACHIEVED = auto()                                 
    ORBIT_FAILED = auto()                                   
    APOGEE_REACHED = auto()                                       
                                                                           
    S2_ORBIT_HOLD = auto()                                         
    S2_DEORBIT = auto()                                                                  
    S2_ENTRY = auto()                                                               
    S2_LANDING = auto()                                                         
                             
    BOOSTER_FLIP = auto()
    BOOSTER_BOOSTBACK = auto()
    BOOSTER_COAST = auto()
    BOOSTER_ENTRY = auto()
    BOOSTER_LANDING = auto()


class MissionManager:
    """
    Manages the current mission phase and transitions using physics-based criteria.
    """

                                                                                 
                                                                                
    NEAR_PAD_HANDOFF_ALTITUDE_M = 7030.0

                                                                              
                                                                                 
                                                                      
                                                                               
                                                                          
    _RECOVERY_FINE_STEP_PHASES = frozenset({
        MissionPhase.BOOSTER_FLIP,
        MissionPhase.BOOSTER_BOOSTBACK,
        MissionPhase.BOOSTER_ENTRY,
        MissionPhase.BOOSTER_LANDING,
    })

    def __init__(self, vehicle_type: str = "ascent", initial_mass: float = None,
                 config: SimulationConfig = None):
        self.vehicle_type = vehicle_type
        self.config = config or create_default_config()
        self.config.validate()
        self.current_phase = MissionPhase.ASCENT

        if vehicle_type == "orbiter":
            self.current_phase = MissionPhase.S2_COAST_TO_APOGEE
        elif vehicle_type == "booster":
            self.current_phase = MissionPhase.BOOSTER_FLIP

        self.meco_time = None
        self.stage_separation_time = None
        self.separation_done = False                                     
        self.apogee_time = None
        self.orbit_insertion_time = None
        self.max_altitude_reached = 0.0
        self._last_radial_velocity = 0.0
        self._phase_entry_time = 0.0                                         
        self._launch_site = rotating_launch_site_eci(0.0, config=self.config)
        self.orbit_failure_reason = None
        self.booster_landing_leg_state = None
        self._initial_mass = float(initial_mass) if initial_mass is not None else C.INITIAL_MASS
                                                                        
        if vehicle_type == "booster" and initial_mass is not None:
            self._initial_propellant = initial_mass - float(self.config.stage1_dry_mass)
        else:
            self._initial_propellant = float(self.config.stage1_prop_mass)

    def set_phase_entry_time(self, t: float) -> None:
        """Set phase entry time when forking a vehicle at mid-mission (e.g. booster at separation)."""
        self._phase_entry_time = t

    def _compute_radial_velocity(self, state: State) -> float:
        """Compute radial velocity: r_dot = (r . v) / |r|."""
        return compute_radial_velocity(state)

    def _compute_horizontal_velocity(self, state: State) -> float:
        """Compute horizontal speed (perpendicular to radial direction)."""
        return compute_horizontal_velocity(state)

    def _check_attitude_aligned(self, state: State, target_dir: np.ndarray,
                                 threshold_deg: float = 15.0) -> bool:
        """
        Check if body +Z axis is aligned within threshold of target direction.

        Physics: The rotation matrix R(q) transforms body -> inertial.
        Body +Z in inertial frame = R @ [0,0,1] = 3rd column of R.
        """
        return check_attitude_aligned(state, target_dir, threshold_deg)

    def _compute_downrange_distance(self, state: State) -> float:
        """
        Compute horizontal (great-circle) distance from launch site.

        Uses the central angle: d = R_earth * arccos(r_hat . r0_hat)
        """
        return compute_downrange_distance(state, self._launch_site)

    def _compute_suicide_burn_altitude(self, state: State, dry_mass: float) -> float:
        """
        Compute suicide-burn ignition altitude using shared recovery estimator.
        """
        return compute_suicide_burn_altitude(
            state,
            self.config.booster_landing_ignition_safety_factor,
        )

    def _ignition_corridor_top(self, state: State) -> float:
        """Altitude ceiling (m AGL) below which the landing burn may ignite.

        Energy-based suicide-burn altitude (scaled by the ignition safety
        factor), capped by the absolute ignition ceiling. Used by the
        BOOSTER_ENTRY -> BOOSTER_LANDING transition test in update().
        """
        burn = estimate_suicide_burn(
            state.r,
            state.v,
            state.m,
            C.LANDING_THRUST,
            safety_factor=self.config.booster_landing_ignition_safety_factor,
        )
        h_ignite = float(burn['burn_altitude'])
        return min(
            self.config.booster_landing_ignition_ceiling_m,
            h_ignite * self.config.booster_landing_ignition_safety_factor,
        )

    def recovery_max_step_dt(self) -> float:
        """Maximum integration step (s) permitted in the current phase.

        Returns ``C.BOOSTER_RECOVERY_MAX_DT`` during the booster's powered/guided
        recovery phases (flip, boostback, entry, landing) and ``+inf`` otherwise.
        Capping the step in those phases keeps the closed-loop targeting (and so
        the landing point) on the validated trajectory for any global dt >= the
        cap; the ballistic coast is left uncapped so a coarse dt still runs fast
        through free-fall.
        """
        if (
            self.vehicle_type == "booster"
            and self.current_phase in self._RECOVERY_FINE_STEP_PHASES
        ):
            return C.BOOSTER_RECOVERY_MAX_DT
        return float("inf")

    def update(self, state: State, dt: float):
        """
        Check for phase transitions based on current vehicle state.

        All transitions use physics-based criteria rather than hardcoded times.

        Args:
            state: Current vehicle state
            dt: Time step
        """
        self.max_altitude_reached = max(self.max_altitude_reached, state.altitude)

        radial_velocity = self._compute_radial_velocity(state)

                                                                               
                                    
                                                                               

                                                     
                                                                                      
                                                      
        if self.current_phase == MissionPhase.ASCENT:
            reserve_kg = self.config.stage1_landing_fuel_reserve_kg
            meco_mass = (
                float(self.config.stage1_dry_mass)
                + float(self.config.stage2_dry_mass)
                + float(self.config.stage2_prop_mass)
                + float(self.config.payload_mass)
                + reserve_kg
            )
            if state.m <= meco_mass:
                s1_prop_burned = self._initial_mass - state.m
                logger.info(f"MECO detected at t={state.t:.2f}s, "
                           f"Alt={state.altitude/1000:.1f}km, "
                           f"V={state.speed:.0f}m/s, "
                           f"S1 prop burned={s1_prop_burned:.0f}kg, "
                           f"Fuel reserved={reserve_kg:.0f}kg")
                self.current_phase = MissionPhase.COAST
                self.meco_time = state.t
                self._phase_entry_time = state.t

                                   
                                                                            
                                                                                       
        elif self.current_phase == MissionPhase.COAST:
            if self.meco_time is not None and (state.t - self.meco_time) > 3.0:
                logger.info(f"Stage Separation at t={state.t:.2f}s, "
                           f"Alt={state.altitude/1000:.1f}km")
                self.current_phase = MissionPhase.STAGE_SEPARATION
                self.stage_separation_time = state.t
                self._phase_entry_time = state.t

                                                
                                                                     
        elif self.current_phase == MissionPhase.STAGE_SEPARATION:
            time_since_sep = state.t - self._phase_entry_time
            if time_since_sep > 2.0:
                logger.info(f"S2 Coast-to-Apogee begins at t={state.t:.2f}s, "
                           f"Alt={state.altitude/1000:.1f}km, "
                           f"V={state.speed:.0f}m/s")
                self.current_phase = MissionPhase.S2_COAST_TO_APOGEE
                self._phase_entry_time = state.t

                                               
                                                                            
                                                                              
                                                                                
                                                                              
                                                               
         
                                                                           
                                                 
        if self.current_phase == MissionPhase.S2_COAST_TO_APOGEE:
                                                                              
                                                                             
                                                                           
                                                                  
                                                                                                   
            above_atmosphere = (
                state.altitude > self.config.orbit_insertion_start_altitude_m
            )
                                                                                 
            # Use the configured S2 entry interface rather than a hard-coded
            # 70 km literal. With the old literal, an insertion that fell short
            # with an apogee below 70 km satisfied neither this disjunct nor
            # above_atmosphere, so the orbiter sat in S2_COAST_TO_APOGEE until
            # max_time. (The default value of this field is 70 km, so default
            # behaviour is unchanged.)
            altitude_falling = (
                radial_velocity < -20.0
                and state.altitude > float(self.config.s2_entry_interface_altitude_m)
            )

            if above_atmosphere or altitude_falling:
                logger.info(f"ORBIT INSERTION BURN START at t={state.t:.2f}s, "
                           f"Alt={state.altitude/1000:.1f}km, "
                           f"V={state.speed:.0f}m/s, "
                           f"V_radial={radial_velocity:.1f}m/s")
                self.current_phase = MissionPhase.ORBIT_INSERTION
                self.orbit_insertion_time = state.t
                self._phase_entry_time = state.t
                return                                                                  

                                           
                                                                   
                                    
        if self.current_phase == MissionPhase.ORBIT_INSERTION:
            orbit = compute_orbit_metrics(state)
            energy = orbit["energy"]
            a_sma = orbit["a_sma"]
            ecc = orbit["ecc"]
            v_mag = orbit["v_mag"]
            perigee_alt = orbit["perigee_alt"]
            apogee_alt = orbit["apogee_alt"]
            s2_prop_exhausted = orbit["s2_prop_exhausted"]
            target_alt = self.config.orbit_target_altitude_m
            alt_tol = self.config.orbit_altitude_tolerance_m
            ecc_max = self.config.orbit_ecc_max
            orbit_ok = (
                energy < 0 and
                abs(perigee_alt - target_alt) <= alt_tol and
                abs(apogee_alt - target_alt) <= alt_tol and
                ecc <= ecc_max
            )

            if orbit_ok:
                perigee = a_sma * (1 - ecc) - C.R_EARTH
                logger.info(f"ORBIT ACHIEVED at t={state.t:.2f}s! "
                           f"Alt={state.altitude/1000:.1f}km, V={v_mag:.0f}m/s, "
                           f"e={ecc:.4f}, a={a_sma/1000:.0f}km, "
                           f"Perigee={perigee/1000:.0f}km, Apogee={apogee_alt/1000:.0f}km")
                self.current_phase = MissionPhase.ORBIT_ACHIEVED
                self._phase_entry_time = state.t

            elif s2_prop_exhausted:
                self.orbit_failure_reason = (
                    f"target miss: perigee={perigee_alt/1000:.1f}km, "
                    f"apogee={apogee_alt/1000:.1f}km, e={ecc:.4f}"
                )
                logger.error(
                    f"S2 PROPELLANT EXHAUSTED at t={state.t:.2f}s "
                    f"(ORBIT INSERTION FAILED), {self.orbit_failure_reason}"
                )
                self.current_phase = MissionPhase.ORBIT_FAILED
                self._phase_entry_time = state.t

                                                                             
                                                                             
            elif (
                state.altitude < 80000.0
                and radial_velocity < -50.0
                and perigee_alt < 100000.0
            ):
                self.orbit_failure_reason = (
                    f"reentered during insertion: alt={state.altitude/1000:.1f}km, "
                    f"perigee={perigee_alt/1000:.1f}km, apogee={apogee_alt/1000:.1f}km, "
                    f"e={ecc:.4f}"
                )
                logger.error(
                    f"ORBIT INSERTION REENTRY at t={state.t:.2f}s "
                    f"(ORBIT INSERTION FAILED), {self.orbit_failure_reason}"
                )
                self.current_phase = MissionPhase.ORBIT_FAILED
                self._phase_entry_time = state.t

            elif (
                state.t - self._phase_entry_time
            ) > float(self.config.orbit_insertion_timeout_s):
                                                                           
                                                                        
                                                                           
                                                                    
                                                                         
                                                                                
                                                                                   
                self.orbit_failure_reason = (
                    f"guidance stall: coast timeout exceeded "
                    f"(perigee={perigee_alt/1000:.1f} km, "
                    f"apogee={apogee_alt/1000:.1f} km, fuel remaining)"
                )
                logger.error(
                    f"ORBIT INSERTION STALL at t={state.t:.2f}s "
                    f"(ORBIT INSERTION FAILED), {self.orbit_failure_reason}"
                )
                self.current_phase = MissionPhase.ORBIT_FAILED
                self._phase_entry_time = state.t

                                                                               
                                                                          
                                                                           
                                                                          
                                                                               
        if self.config.enable_s2_recovery and self.vehicle_type == "orbiter":
            if self.current_phase == MissionPhase.ORBIT_ACHIEVED:
                self.current_phase = MissionPhase.S2_ORBIT_HOLD
                self._phase_entry_time = state.t
                logger.info(f"S2 Orbit Hold begins at t={state.t:.2f}s")
            elif self.current_phase == MissionPhase.S2_ORBIT_HOLD:
                hold_elapsed = (state.t - self._phase_entry_time) >= self.config.s2_orbit_hold_time_s
                at_apoapsis = self._last_radial_velocity > 0.0 and radial_velocity <= 0.0
                if hold_elapsed and at_apoapsis:
                    self.current_phase = MissionPhase.S2_DEORBIT
                    self._phase_entry_time = state.t
                    logger.info(f"S2 Deorbit Burn start at t={state.t:.2f}s")
            elif self.current_phase == MissionPhase.S2_DEORBIT:
                                                                             
                                                                            
                                                                
                if (
                    state.altitude < self.config.s2_entry_interface_altitude_m
                    and radial_velocity < 0.0
                ):
                    orbit = compute_orbit_metrics(state)
                    self.current_phase = MissionPhase.S2_ENTRY
                    self._phase_entry_time = state.t
                    logger.info(
                        f"S2 Entry Interface at t={state.t:.2f}s, "
                        f"Alt={state.altitude/1000:.1f}km, V={state.speed:.0f}m/s "
                        f"(perigee={orbit['perigee_alt']/1000:.1f}km)"
                    )
            elif self.current_phase == MissionPhase.S2_ENTRY:
                                                                           
                                                                               
                                                                            
                                                     
                if radial_velocity < 0.0 and state.altitude < 15000.0:
                    self.current_phase = MissionPhase.S2_LANDING
                    self._phase_entry_time = state.t
                    logger.info(
                        f"S2 Landing Burn at t={state.t:.2f}s, "
                        f"Alt={state.altitude/1000:.1f}km, V={state.speed:.0f}m/s"
                    )

                                                                               
                                                      
                                                                               

        if self.vehicle_type == "booster":

                               
                                                                           
                                                                            
                                                                      
            if self.current_phase == MissionPhase.BOOSTER_FLIP:
                v_norm = np.linalg.norm(state.v)
                if v_norm > 1.0:
                    retrograde = -state.v / v_norm
                    aligned = self._check_attitude_aligned(state, retrograde, 15.0)
                else:
                    aligned = True                                                   

                                                                                      
                time_in_phase = state.t - self._phase_entry_time
                min_flip_time = max(float(self.config.booster_flip_min_time_s), 0.0)
                if (aligned and time_in_phase > min_flip_time) or time_in_phase > 30.0:
                    logger.info(f"Booster Flip Complete at t={state.t:.2f}s "
                               f"(attitude aligned: {aligned})")
                    self.current_phase = MissionPhase.BOOSTER_BOOSTBACK
                    self._phase_entry_time = state.t

                                
                                                                             
                                                                              
                                                                     
             
                                                                               
             
                                                           
             
                                                                        
                                                                            
                                                                       
                                                                          
                                                                          
                               
            elif self.current_phase == MissionPhase.BOOSTER_BOOSTBACK:
                r_hat = state.r / max(np.linalg.norm(state.r), 1.0)
                v_horiz = state.v - np.dot(state.v, r_hat) * r_hat
                v_horiz_mag = float(np.linalg.norm(v_horiz))

                                                                                             
                                                                 
                                                                                    
                                                                                       
                target_downrange_km = self.config.booster_landing_target_downrange_km
                _lead_mm = compute_powered_descent_lead_time(
                    state.r, state.v, state.m, self.config
                ) + 22.4
                targeting = estimate_recovery_targeting(
                    state.r,
                    state.v,
                    state.t,
                    target_downrange_km,
                    config=self.config,
                    lead_time_s=_lead_mm,
                )
                t_coast_est = targeting.coast_time_s

                                                                               
                                                                        
                                                                                 
                                                                            
                                                                           
                site_dist = targeting.site_distance_m
                v_toward_site = targeting.v_toward_site_mps
                v_return_needed = targeting.v_return_needed_mps
                impact = estimate_ballistic_impact_to_pad(
                    state.r,
                    state.v,
                    state.t,
                    target_downrange_km,
                    self.config,
                    lead_time_s=0.0,
                    mass_kg=state.m,
                    aero_mode=self.current_phase.name,
                )

                                                                              
                                                                            
                                                                               
                                                                              
                                                                        
                                                                        
                landing_site_now = target_landing_site_eci(
                    state.t,
                    target_downrange_km,
                    config=self.config,
                )
                r_to_pad_now = landing_site_now - state.r
                r_to_pad_now_horiz = r_to_pad_now - np.dot(r_to_pad_now, r_hat) * r_hat
                pad_dist_now = float(np.linalg.norm(r_to_pad_now_horiz))
                if pad_dist_now > 100.0:
                    toward_pad_now = r_to_pad_now_horiz / pad_dist_now
                    v_ground = compute_ground_relative_velocity(
                        state.r,
                        state.v,
                    )
                    v_ground_horiz = v_ground - np.dot(v_ground, r_hat) * r_hat
                    v_pad_closure = float(np.dot(v_ground_horiz, toward_pad_now))
                else:
                    v_pad_closure = 0.0
                t_return_ground = max(t_coast_est * 1.00, 60.0)
                v_pad_needed = float(np.clip(pad_dist_now / t_return_ground, 20.0, 250.0))

                                                                             
                                                                 
                 
                                                                     
                                                                            
                                                                         
                                                                           
                                                                            
                                                      
                                                                            
                                                                             
                                                                         
                                                                        
                near_pad_rtls = is_near_pad_target(self.config)
                                                                                
                                                                              
                                                                           
                                                                             
                                                           
                                                                        
                                                                                  
                                                                             
                                                                                
                                                            
                                                                               
                                                                             
                                                                              
                                                                         
                impact_corridor_ready = (
                    impact.miss_distance_m
                    <= float(self.config.booster_boostback_impact_corridor_m)
                )
                boostback_elapsed = state.t - self._phase_entry_time
                min_boostback_s = 15.0
                boostback_min_time_met = boostback_elapsed >= min_boostback_s
                lower_return_ratio = 0.92
                upper_return_ratio = 1.25
                velocity_on_target = (
                    v_toward_site > 0.0 and
                    v_toward_site >= v_return_needed * lower_return_ratio
                    and v_toward_site <= v_return_needed * upper_return_ratio
                )
                if near_pad_rtls:
                    boostback_complete = (
                        velocity_on_target
                        and impact_corridor_ready
                        and boostback_min_time_met
                    )
                else:
                    ground_return_ready = v_pad_closure >= 1.50 * v_pad_needed
                    boostback_complete = (
                        velocity_on_target
                        and ground_return_ready
                        and impact_corridor_ready
                        and boostback_min_time_met
                    )

                                                
                propellant_remaining = booster_propellant_remaining(state.m, self.config)
                min_after_boostback = booster_min_propellant_after_boostback(self.config)
                                                                                   
                                                                                 
                                                                                     
                propellant_used = max(0.0, self._initial_propellant - propellant_remaining)
                budget_used = propellant_used >= self.config.booster_boostback_budget_kg
                fuel_guard = propellant_remaining <= min_after_boostback

                if boostback_complete or budget_used or fuel_guard:
                    reason = ("impact on target" if (boostback_complete and near_pad_rtls) else
                              "velocity on target" if boostback_complete else
                              "boostback budget" if budget_used else "reserve guard")
                    logger.info(
                        f"Boostback Complete at t={state.t:.2f}s ({reason}), "
                        f"V_horiz={v_horiz_mag:.0f}m/s, "
                        f"V_toward_site={v_toward_site:.0f}m/s, "
                        f"V_needed={v_return_needed:.0f}m/s, "
                        f"V_pad={v_pad_closure:.0f}m/s, "
                        f"V_pad_needed={v_pad_needed:.0f}m/s, "
                        f"impact_miss={impact.miss_distance_m/1000:.1f}km, "
                        f"site_dist={site_dist/1000:.1f}km, "
                        f"Vr={radial_velocity:.0f}m/s, "
                        f"prop={propellant_remaining:.0f}kg"
                    )
                    self.current_phase = MissionPhase.BOOSTER_COAST
                    self._phase_entry_time = state.t

                            
                                                                                
                                                                             
                                                                   
            elif self.current_phase == MissionPhase.BOOSTER_COAST:
                                                                  
                entry_interface = self.config.booster_entry_interface_altitude_m
                if state.altitude < entry_interface and radial_velocity < 0.0:
                    logger.info(f"Booster Entry Interface at t={state.t:.2f}s, "
                               f"Alt={state.altitude/1000:.1f}km, "
                               f"V={state.speed:.0f}m/s")
                    self.current_phase = MissionPhase.BOOSTER_ENTRY
                    self._phase_entry_time = state.t

                              
                                                                     
                                                                           
                                                                        
                                                                     
             
                                                                               
                                                                             
                                                                            
                                                                              
                                                                             
                                                                             
                                                                           
                                                                               
                                                                              
                                                                                 
            elif self.current_phase == MissionPhase.BOOSTER_ENTRY:
                                                                                    
                                                                                  
                                                                                
                                                                         
                                                                                     
                                                                              
                 
                                                                                    
                                                                                  
                                                                                 
                                                                                  
                burn = estimate_suicide_burn(
                    state.r,
                    state.v,
                    state.m,
                    C.LANDING_THRUST,
                    safety_factor=self.config.booster_landing_ignition_safety_factor,
                )
                h_ignite = float(burn['burn_altitude'])

                                                                            
                                                                              
                                                                               
                                                                              
                                                                             
                                                                             
                                                                             
                                                                             
                                                                               
                                                                     
                ignition_corridor_top = self._ignition_corridor_top(state)
                landing_trigger = (
                    burn['ignite'] and
                    radial_velocity < 0.0 and                      
                    state.altitude <= ignition_corridor_top
                )
                near_pad_handoff_ready = False
                if (
                    is_near_pad_target(self.config)
                    and radial_velocity < 0.0
                    and state.altitude <= self.NEAR_PAD_HANDOFF_ALTITUDE_M
                ):
                    propellant_remaining = booster_propellant_remaining(state.m, self.config)
                    impact = estimate_ballistic_impact_to_pad(
                        state.r,
                        state.v,
                        state.t,
                        self.config.booster_landing_target_downrange_km,
                        self.config,
                        mass_kg=state.m,
                        aero_mode=self.current_phase.name,
                    )
                    near_pad_handoff_ready = (
                        propellant_remaining
                        > self.config.booster_landing_reserve_kg * 0.75
                        and impact.miss_distance_m
                        <= self.config.booster_pad_tolerance_m
                        and impact.time_to_impact_s >= 8.0
                    )

                if landing_trigger or near_pad_handoff_ready:
                    logger.info(f"Booster Landing Phase at t={state.t:.2f}s, "
                               f"Alt={state.altitude/1000:.1f}km, "
                               f"h_ignite={h_ignite:.0f}m")
                    self.current_phase = MissionPhase.BOOSTER_LANDING
                    self._phase_entry_time = state.t
        self._last_radial_velocity = radial_velocity

    def get_phase(self) -> MissionPhase:
        return self.current_phase

    @property
    def separation_time(self) -> float | None:
        """Return the actual recorded time of the stage-separation event.

        Returns None if separation never occurred (e.g. an ascent-only run that
        ended before MECO/separation). Callers must guard against None.
        """
        return self.stage_separation_time
