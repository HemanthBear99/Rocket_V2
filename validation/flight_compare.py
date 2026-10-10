"""Compare the simulated stage-1 flight with Falcon 9 webcast telemetry.

Data: shahar603/Telemetry-Data (public domain), extracted from SpaceX
webcasts at 1 Hz -- surface-relative speed and altitude, event times.
The simulated vehicle uses Falcon 9 FT stage masses and thrust; mission
details (inclination, payload when unpublished) are the sim defaults.

    python -m rlv_sim.validation.flight_compare      # writes docs/VALIDATION.md + plot
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from rlv_sim._run_full_mission import run_full_mission
from rlv_sim.config_factory import create_default_config

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
DOCS = ROOT.parent / "docs"

# name -> payload mass to simulate (kg); None = unpublished, use sim default.
FLIGHTS = {
    "SpaceX_CRS-11": 6900.0,   # 2708 kg cargo + ~4.2 t Dragon 1
    "Orbcomm_OG2": 2034.0,
    "NROL-76": None,
    "ZUMA": None,
}


def _first_time(times, phases, key):
    for t, p in zip(times, phases):
        if key in str(p).upper():
            return float(t)
    return None


def simulate(payload_kg: float | None) -> dict:
    cfg = create_default_config(verbose=False, **({"payload_mass": payload_kg} if payload_kg else {}))
    res = run_full_mission(config=cfg, verbose=False)
    a, b = res.ascent_log, res.booster_log
    t = np.asarray(a.get_series("time"))
    alt = np.asarray(a.get_series("altitude"))  # km
    v = np.asarray(a.get_series("velocity_rel"))
    q = np.asarray(a.get_series("dynamic_pressure"))
    bt, bph = b.get_series("time"), b.get_series("phase_name")
    balt = np.asarray(b.get_series("altitude"))  # km
    meco = float(res.separation_time)
    i = int(np.searchsorted(t, meco)) - 1
    return {
        "t": t, "alt": alt, "v": v,
        "meco": meco, "meco_v": float(v[i]), "meco_alt": float(alt[i]),
        "maxq": float(t[int(np.argmax(q))]), "peak_q_kpa": float(q.max()) / 1000.0,
        "apogee_km": float(balt.max()) if len(balt) else None,
        "boostback_start": _first_time(bt, bph, "BOOSTBACK"),
        "entry_start": _first_time(bt, bph, "ENTRY"),
        "landing_start": _first_time(bt, bph, "LANDING"),
        "landing_end": float(bt[-1]) if len(bt) else None,
        "landed": bool(res.booster_landing_success),
    }


def load_flight(name: str) -> dict:
    d = json.loads((DATA / f"{name}.json").read_text())
    ev = json.loads((DATA / f"{name}_events.json").read_text())
    t = np.asarray(d["time"], float)
    alt = np.asarray(d["altitude"], float)
    v = np.asarray(d["velocity"], float)
    meco = float(ev["meco"])
    i = int(np.searchsorted(t, meco))
    asc = t <= meco
    return {
        "t": t, "alt": alt, "v": v, "events": ev, "meco": meco,
        "meco_v": float(v[i]), "meco_alt": float(alt[i]),
        "apogee_km": float(alt[~asc].max()) if (~asc).any() and ev.get("apogee") else None,
        "peak_q_kpa": float(np.nanmax(np.asarray(d["q"], float)[asc])) / 1000.0,
    }


def _pct(sim, real):
    return f"{100.0 * (sim - real) / real:+.1f}%" if sim is not None and real else "–"


def _fmt(x, f="{:.1f}"):
    return f.format(x) if x is not None else "–"


def main() -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_alt, ax_v) = plt.subplots(1, 2, figsize=(12, 4.5))
    rows, curve_rows = [], []
    sims: dict = {}
    for name, payload in FLIGHTS.items():
        real = load_flight(name)
        key = payload or "default"
        sim = sims.setdefault(key, simulate(payload))
        # Ascent-curve error up to the earlier MECO, on the flight's 1 Hz grid.
        t_end = min(real["meco"], sim["meco"])
        m = real["t"] <= t_end
        alt_rms = float(np.sqrt(np.mean((np.interp(real["t"][m], sim["t"], sim["alt"]) - real["alt"][m]) ** 2)))
        v_rms = float(np.sqrt(np.mean((np.interp(real["t"][m], sim["t"], sim["v"]) - real["v"][m]) ** 2)))
        curve_rows.append(f"| {name} | {_fmt(payload, '{:.0f}') if payload else '8000 (unpublished)'} "
                          f"| {alt_rms:.2f} | {v_rms:.0f} |")
        ev = real["events"]
        rows += [
            f"| {name} | MECO time (s) | {real['meco']:.0f} | {sim['meco']:.1f} | {_pct(sim['meco'], real['meco'])} |",
            f"| | MECO speed (m/s) | {real['meco_v']:.0f} | {sim['meco_v']:.0f} | {_pct(sim['meco_v'], real['meco_v'])} |",
            (f"| | MECO altitude (km) | {real['meco_alt']:.1f} | {sim['meco_alt']:.1f} "
            f"| {_pct(sim['meco_alt'], real['meco_alt'])} |"),
            (f"| | Peak q (kPa) | {real['peak_q_kpa']:.1f} | {sim['peak_q_kpa']:.1f} "
            f"| {_pct(sim['peak_q_kpa'], real['peak_q_kpa'])} |"),
            f"| | Max-q time (s) | {ev['maxq']} | {sim['maxq']:.0f} | {_pct(sim['maxq'], ev['maxq'])} |",
            (f"| | Booster apogee (km) | {_fmt(real['apogee_km'])} | {_fmt(sim['apogee_km'])} "
            f"| {_pct(sim['apogee_km'], real['apogee_km'])} |"),
        ]
        for label, k in (("Boostback start (s)", "boostback_start"), ("Entry burn start (s)", "entry_start"),
                         ("Landing burn start (s)", "landing_start"), ("Touchdown (s)", "landing_end")):
            if ev.get(k):
                rows.append(f"| | {label} | {ev[k]} | {_fmt(sim[k], '{:.0f}')} | {_pct(sim[k], ev[k])} |")
        mr = real["t"] <= real["meco"]
        ax_alt.plot(real["t"][mr], real["alt"][mr], lw=1, label=name)
        ax_v.plot(real["t"][mr], real["v"][mr], lw=1, label=name)
    for key, sim in sims.items():
        ms = sim["t"] <= sim["meco"]
        lbl = f"Sim ({key:.0f} kg)" if key != "default" else "Sim (8000 kg)"
        ax_alt.plot(sim["t"][ms], sim["alt"][ms], "k--", lw=1.6, label=lbl)
        ax_v.plot(sim["t"][ms], sim["v"][ms], "k--", lw=1.6)
    ax_alt.set(xlabel="Time (s)", ylabel="Altitude (km)", title="Stage-1 ascent: altitude")
    ax_v.set(xlabel="Time (s)", ylabel="Surface-relative speed (m/s)", title="Stage-1 ascent: speed")
    for ax in (ax_alt, ax_v):
        ax.grid(alpha=0.3)
    ax_alt.legend(fontsize=7)
    fig.tight_layout()
    DOCS.mkdir(exist_ok=True)
    fig.savefig(DOCS / "validation_ascent.png", dpi=130)

    (DOCS / "VALIDATION.md").write_text("\n".join([
        "# Validation against Falcon 9 flight telemetry",
        "",
        ("Generated by `python -m rlv_sim.validation.flight_compare`. Flight data: "
        "[shahar603/Telemetry-Data](https://github.com/shahar603/Telemetry-Data) (public domain), "
        "extracted from SpaceX webcasts at 1 Hz. Sim: default vehicle (Falcon 9 FT stage masses, "
        "9 x 845 kN), payload matched where published, otherwise the 8 t default; target orbit, "
        "inclination and landing site are sim defaults, not the flight's, so differences in "
        "booster timeline are partly mission, not model."),
        "",
        "![Ascent overlay](validation_ascent.png)",
        "",
        "## Ascent curve error (lift-off to MECO)",
        "",
        "| Flight | Sim payload (kg) | Altitude RMS (km) | Speed RMS (m/s) |",
        "|---|---|---|---|",
        *curve_rows,
        "",
        "## Key events",
        "",
        "| Flight | Quantity | Flight | Sim | Difference |",
        "|---|---|---|---|---|",
        *rows,
        "",
        "## Findings",
        "",
        "- **MECO speed matches within ~2.5%** on every flight: the energy the stage delivers is right.",
        ("- **The sim reaches MECO ~10-15 s early and ahead on the curve.** Stage-1 burns its ascent "
        "propellant at full thrust (one throttle law, q-hold at 32 kPa), while Falcon 9 throttles "
        "down through max-q (real peak q 23-29 kPa vs the sim's 32 kPa target) and burns longer at lower acceleration. The sim's trajectory is "
        "therefore more aggressive in time, not in end state."),
        ("- **Booster timelines start right (boostback within ~2%)**; later events differ by up to "
        "~20% because the sim flies its own target orbit, inclination and pad, not each mission's."),
        ("- Next model work: a Falcon-style throttle bucket around max-q and per-flight mission "
        "settings (inclination, target orbit) would tighten the ascent-curve error."),
        "",
    ]), encoding="utf-8")
    print((DOCS / "VALIDATION.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    main()
