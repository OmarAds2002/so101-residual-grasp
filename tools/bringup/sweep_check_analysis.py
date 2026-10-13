import csv, numpy as np, matplotlib.pyplot as plt

rows = list(csv.DictReader(open("results/sweep_checks_raw.csv")))
a = np.array(
    [
        [
            float(r["cmd_speed"]),
            float(r["target"]),
            float(r["ticks"]),
            abs(float(r["load"])),
        ]
        for r in rows
    ]
)
speed, lag, load = a[:, 0], np.abs(a[:, 1] - a[:, 2]), a[:, 3]

for name, x in [("speed", speed), ("lag", lag)]:
    xs, ys = [], []
    for s in sorted(set(speed)):
        m = speed == s
        xs.append(np.median(x[m]))
        ys.append(np.median(load[m]))
    xs, ys = np.array(xs), np.array(ys)
    k, c = np.polyfit(xs, ys, 1)
    resid = ys - (k * xs + c)
    print(
        f"load vs {name:<6}: slope {k:8.4f}  intercept {c:8.2f}  "
        f"max resid {np.abs(resid).max():6.2f}"
    )
    plt.plot(xs, ys, "o-", label=f"vs {name}")

plt.xlabel("speed or lag")
plt.ylabel("|load|")
plt.legend()
plt.grid(alpha=0.3)
plt.show()
