# System-identification scripts: fitting the MJCF to recorded trajectories (writeup: ../bringup/results/08_mjcf_fit.md)

| Script | Does | Input / output |
|---|---|---|
| fit_mjcf.py | Fits kp, damping and frictionloss by replaying recorded commands through MuJoCo, then validates on held-out runs | in: ../bringup/results/trajectories/*.csv and src/p4_common/mjcf/so101.xml; out: terminal only |
| plot_one.py | Plots sim against real for one run, to diagnose a bad fit before tuning | in: one trajectory CSV; out: diagnose.png in the current directory |
| sim_only.py | Sanity-checks the sim with no recording: hold at 0, step pan to +0.4 rad, step with gravity off | no input; out: terminal only |
