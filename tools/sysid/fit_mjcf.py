#!/usr/bin/env python3
"""Fit kp, damping and frictionloss against recorded real trajectories.

Replays each recording's COMMANDS through MuJoCo and compares the sim's
measured positions to the real arm's. An optimiser searches the three
parameters for the combination that matches best.

Cost, per run, per moving joint:
    w_p*(pos_err/pos_sd)^2 + w_v*(vel_err/vel_sd)^2 + w_l*(load_err/load_sd)^2
Each term divided by its own spread so none dominates by its units.
Runs are averaged, NOT pooled -- otherwise the 2400-sample runs outvote
the 320-sample one, which is the only run probing the top of the band.

Velocity error is in there because position error is slow to react; a
small sustained velocity mismatch takes a long time to show as a gap.
Velocity error is largest exactly at reversals and transients, which is
where friction and kp show themselves.

Dead time: the real arm takes ~65 ms to start moving, MuJoCo starts at
once, so recordings are shifted. Swept over 45-85 ms: cost was flat, so
the shift is not absorbing a timing error and 65 stands.

PHYSICAL BOUNDS, enforced during the search rather than checked after:
  stall torque              ~1.9 N*m
  arm weight at horizontal  ~1.0 N*m (measured: load 72-80 units)
  the arm FALLS when torque is off  -> friction < gravity torque
  it holds within 2 ticks when on   -> kp is order 100 N*m/rad

Velocity for the damping bound is the 90th percentile, not the maximum.
Peak velocity happens for an instant at the top of a swing while the
joint is already decelerating; momentum covers it. Sustained velocity is
what damping has to be paid for continuously.

MULTI-START. Three starting points, best result wins. If all three land
together the parameters are identified. If they land apart with similar
costs, damping and friction cannot be separated from this data -- which
is a finding about the recordings, not a failure of the fit.
"""

import csv
from pathlib import Path
import numpy as np
import yaml
import mujoco
from scipy.optimize import minimize

HERE     = Path(__file__).resolve().parent
TRAJ     = HERE.parent / "bringup/results/trajectories"
MJCF     = HERE.parent.parent / "src/p4_common/mjcf/so101.xml"
CAL_PATH = HERE.parent.parent / "src/p4_common/config/calibration.yaml"

_cal    = yaml.safe_load(open(CAL_PATH))["servos"]
JOINTS  = {v["servo_id"]: name for name, v in _cal.items()}
OFFSETS = {v["servo_id"]: v["offset"] for name, v in _cal.items()}
SIGNS   = {v["servo_id"]: v["sign"]   for name, v in _cal.items()}

HOME_TICKS  = {1: 2096, 2: 2000, 3: 1875, 4: 2020, 5: 2074, 6: 2266}
TICK2RAD    = 2 * np.pi / 4096
LOAD_PER_NM = 74.3           # measured S05, lever 0.35 m +/- 2 cm

STALL_NM     = 1.9
GRAVITY_NM   = 1.0
DEAD_TIME_MS = 65.0

TIMESTEP   = 0.001           # 1 ms; 200 Hz control -> decimation 5 exactly
DECIMATION = 5

W_POS, W_VEL, W_LOAD = 1.0, 1.0, 1.0


def ticks_to_rad(ticks, sid):
    return SIGNS[sid] * (np.asarray(ticks, float) - OFFSETS[sid]) * TICK2RAD


def load_run(path):
    """Returns {servo_id: dict of arrays}, already shifted for dead time."""
    rows = list(csv.DictReader(open(path)))
    by_id = {}
    for r in rows:
        sid = int(r["servo_id"])
        by_id.setdefault(sid, []).append(
            (float(r["t_ms"]), float(r["commanded"]),
             float(r["measured"]), float(r["load"])))

    out = {}
    for sid, v in by_id.items():
        a = np.array(sorted(v))
        t, cmd, meas, load = a[:, 0], a[:, 1], a[:, 2], a[:, 3]

        # the command at time t produced motion at t + dead time, so
        # compare each command against the measurement 65 ms later
        t_shift = t - DEAD_TIME_MS
        keep = t_shift >= t[0]
        out[sid] = dict(
            t    = t[keep] / 1000.0,
            cmd  = ticks_to_rad(np.interp(t_shift[keep], t, cmd), sid),
            meas = ticks_to_rad(meas[keep], sid),
            load = load[keep] / LOAD_PER_NM,
        )
    return out


def simulate(model, data, run, params):
    """Replay a run's commands in MuJoCo. Returns sim pos and torque."""
    kp, damping, friction = params

    for jname in JOINTS.values():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        model.dof_damping[model.jnt_dofadr[jid]]      = damping
        model.dof_frictionloss[model.jnt_dofadr[jid]] = friction
    model.actuator_gainprm[:, 0] =  kp
    model.actuator_biasprm[:, 1] = -kp
    model.actuator_forcerange[:] = [-STALL_NM, STALL_NM]
    model.jnt_actfrcrange[:]     = [-STALL_NM, STALL_NM]

    mujoco.mj_resetData(model, data)

    # joints not moving in this run are held at the recording's HOME pose
    for sid, jname in JOINTS.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        adr = model.jnt_qposadr[jid]
        data.qpos[adr] = (run[sid]["meas"][0] if sid in run
                          else ticks_to_rad(HOME_TICKS[sid], sid))
        data.ctrl[jid] = data.qpos[adr]
    mujoco.mj_forward(model, data)

    sids = sorted(run)
    jids = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINTS[s])
            for s in sids}
    n    = len(run[sids[0]]["t"])
    pos  = {s: np.zeros(n) for s in sids}
    tau  = {s: np.zeros(n) for s in sids}

    for k in range(n):
        for s in sids:
            data.ctrl[jids[s]] = run[s]["cmd"][k]
        for _ in range(DECIMATION):
            mujoco.mj_step(model, data)
        for s in sids:
            pos[s][k] = data.qpos[model.jnt_qposadr[jids[s]]]
            tau[s][k] = data.actuator_force[jids[s]]

    return pos, tau


def run_cost(model, data, run, params):
    pos, tau = simulate(model, data, run, params)
    costs = []
    for s in sorted(run):
        r = run[s]
        real_vel = np.gradient(r["meas"], r["t"])

        pos_err = pos[s] - r["meas"]
        vel_err = np.gradient(pos[s], r["t"]) - real_vel
        tau_err = np.abs(tau[s]) - np.abs(r["load"])

        p_sd = max(r["meas"].std(),          1e-4)
        v_sd = max(real_vel.std(),           1e-3)
        l_sd = max(np.abs(r["load"]).std(),  1e-3)

        costs.append(W_POS  * np.mean((pos_err / p_sd) ** 2)
                   + W_VEL  * np.mean((vel_err / v_sd) ** 2)
                   + W_LOAD * np.mean((tau_err / l_sd) ** 2))
    return float(np.mean(costs))


def check_physical(kp, damping, friction, vel):
    """Report any value the hardware could not produce."""
    msgs = []
    if damping * vel > STALL_NM:
        msgs.append(f"damping {damping:.3f} needs {damping*vel:.2f} N*m "
                    f"at {vel:.2f} rad/s, stall is {STALL_NM}")
    if friction > GRAVITY_NM:
        msgs.append(f"frictionloss {friction:.3f} > gravity torque "
                    f"{GRAVITY_NM} -- the arm would not fall when limp, "
                    f"but it does")
    if not 5.0 < kp < 2000.0:
        msgs.append(f"kp {kp:.1f} outside 5..2000; holding within 2 ticks "
                    f"against ~0.3 N*m friction implies order 100")
    return msgs


def fit(name, run_files):
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    model.opt.timestep = TIMESTEP
    data = mujoco.MjData(model)

    runs = [load_run(TRAJ / f) for f in run_files]
    vels = np.concatenate([np.abs(np.gradient(r[s]["meas"], r[s]["t"]))
                           for r in runs for s in r])
    vel_p90 = float(np.percentile(vels, 90))
    vel_max = float(vels.max())

    # bounds straight from the physics, so the search cannot leave it
    lo = np.log([5.0,    0.001,                0.001])
    hi = np.log([2000.0, STALL_NM / vel_p90,   GRAVITY_NM])

    def objective(x):
        kp, damping, friction = np.exp(np.clip(x, lo, hi))
        return float(np.mean([run_cost(model, data, r,
                                       (kp, damping, friction))
                              for r in runs]))

    print(f"\n=== {name} ===")
    print(f"  runs: {', '.join(run_files)}")
    print(f"  velocity  p90 {vel_p90:.2f}   max {vel_max:.2f} rad/s")
    print(f"  bounds    kp 5..2000   damping <{np.exp(hi[1]):.3f}"
          f"   friction <{GRAVITY_NM}")

    starts = [np.log([20.0,  0.10, 0.05]),
              np.log([100.0, 1.00, 0.30]),
              np.log([500.0, 0.05, 0.60])]

    best = None
    for k, x0 in enumerate(starts):
        r = minimize(objective, np.clip(x0, lo, hi), method="Nelder-Mead",
                     options=dict(maxiter=600, xatol=1e-4, fatol=1e-5))
        kp_, d_, f_ = np.exp(np.clip(r.x, lo, hi))
        print(f"    start {k}: cost {r.fun:7.4f} ({r.nit:3d} iters)  "
              f"kp {kp_:8.1f}  damping {d_:.4f}  friction {f_:.4f}")
        if best is None or r.fun < best.fun:
            best = r

    kp, damping, friction = np.exp(np.clip(best.x, lo, hi))
    print(f"  BEST cost {best.fun:.4f}")
    print(f"    kp           {kp:10.3f}  N*m/rad")
    print(f"    damping      {damping:10.4f}  N*m*s/rad")
    print(f"    frictionloss {friction:10.4f}  N*m")

    problems = check_physical(kp, damping, friction, vel_p90)
    if problems:
        print("  PHYSICALLY IMPOSSIBLE:")
        for m in problems:
            print(f"    - {m}")
    else:
        print("  physical checks pass")
        print(f"    damping at p90 speed: {damping*vel_p90:.3f} N*m "
              f"(stall {STALL_NM})")
        print(f"    friction vs gravity : {friction:.3f} / {GRAVITY_NM} N*m")

    return dict(kp=kp, damping=damping, frictionloss=friction,
                cost=best.fun, ok=not problems)


def validate(name, params, run_files):
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    model.opt.timestep = TIMESTEP
    data = mujoco.MjData(model)
    p = (params["kp"], params["damping"], params["frictionloss"])

    print(f"\n--- validation: {name} ---")
    for f in run_files:
        run = load_run(TRAJ / f)
        c = run_cost(model, data, run, p)
        pos, _ = simulate(model, data, run, p)
        rms = np.sqrt(np.mean([(pos[s] - run[s]["meas"]) ** 2 for s in run]))
        print(f"  {f:<22} cost {c:8.4f}   rms {rms/TICK2RAD:7.1f} ticks")


if __name__ == "__main__":
    pan = fit("shoulder_pan", [
        "fit_pan_ramp_slow.csv", "fit_pan_ramp_med.csv",
        "fit_pan_sine.csv",      "fit_pan_ramp_fast.csv"])

    lift = fit("shoulder_lift", [
        "fit_lift_sine.csv", "fit_lift_steps.csv"])

    validate("pan",  pan,  ["val_pan_chirp.csv"])
    validate("lift", lift, ["val_multi.csv"])
    validate("elbow (pan's params, NOT fitted for elbow)",
             pan, ["val_elbow_sine.csv"])

    print("\nfit cost vs validation cost: if validation is much worse,")
    print("the parameters fitted the recordings rather than the robot.")
    print("three starts landing together: identified.")
    print("three starts landing apart at similar cost: not separable")
    print("from this data -- needs recordings over a wider speed range.")