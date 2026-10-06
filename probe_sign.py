"""Measure the actual touchdown miss direction under +12 m/s wind."""
import numpy as np

from rlv_sim._run_full_mission import run_full_mission
from rlv_sim.config_factory import create_demo_config
from rlv_sim.recovery import target_landing_site_eci

cfg = create_demo_config(runtime_wind_offset_mps=12.0)
res = run_full_mission(config=cfg, verbose=False)

st = res.booster_final_state
r = st.r
rhat = r / np.linalg.norm(r)
east = np.cross([0.0, 0.0, 1.0], rhat)
east /= np.linalg.norm(east)

tgt = target_landing_site_eci(st.t, 0.0, config=cfg)
d = r - tgt
east_m = float(np.dot(d, east))
total_m = float(np.linalg.norm(d))

print("booster:", res.booster_reason)
print("miss magnitude = %.1f m" % total_m)
print("east component (downrange) = %+.1f m" % east_m)
print()
if east_m > 0:
    print("=> vehicle landed EAST (downrange) of target: OVERSHOT.")
    print("   Target should move further EAST (positive offset) to meet it.")
else:
    print("=> vehicle landed WEST (uprange) of target: FELL SHORT.")
    print("   Target should move WEST (negative offset) to meet it.")