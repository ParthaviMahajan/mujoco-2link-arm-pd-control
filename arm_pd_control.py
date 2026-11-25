# table["Link1 pos"].iloc[-1] is the last value in a column
# table["Link1 pos"].max() is the biggest value
# table["Link1 pos"].idxmax() is the row number where the max occurs, and table["Time"].iloc[row_number] then gives the time of that row
# print(f"label: {value:.2f}") prints a number rounded to 2 decimals

import mujoco
import pandas as pd
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path("arm.xml")

def run(Kp, Kd, target1=0.0, target2=0.0, seconds=5.0):
    data = mujoco.MjData(model)          # fresh state for every run
    n_steps = int(round(seconds / model.opt.timestep))
    rows = []
    for i in range(n_steps):
        angle1, angle2 = data.qpos[0], data.qpos[1]
        vel1, vel2 = data.qvel[0], data.qvel[1]
        torque1 = Kp * (target1 - angle1) - Kd * vel1
        torque2 = Kp * (target2 - angle2) - Kd * vel2
        data.ctrl[0] = torque1
        data.ctrl[1] = torque2
        mujoco.mj_step(model, data)
        rows.append({
            "Time": data.time,
            "Link1 pos": data.qpos[0], "Link2 pos": data.qpos[1],
            "Link1 vel": data.qvel[0], "Link2 vel": data.qvel[1],
            "Torque1": torque1, "Torque2": torque2,
        })
    return pd.DataFrame(rows)

def report(table, label, target1=0.0, target2=0.0):
    print(label)
    for name, target in [("Link1", target1), ("Link2", target2)]:
        column = table[name + " pos"]
        final = column.iloc[-1]
        peak = column.max()
        peak_time = table["Time"].iloc[column.idxmax()]
        print(f"  {name}")
        print(f"    Steady-state error: {(final - target) * 57.3:.2f} deg")
        print(f"    Peak angle: {peak * 57.3:.2f} deg at t = {peak_time:.2f} s")
        print(f"    Overshoot: {(peak - final) * 57.3:.2f} deg ({(peak - final) / final * 100:.1f} %)")

gains = [(50.0, 5.0), (100.0, 10.0), (200.0, 10.0)]
results = {}
for Kp, Kd in gains:
    label = f"Kp={Kp:.0f}, Kd={Kd:.0f}"
    results[label] = run(Kp, Kd)
    report(results[label], label)
    results[label].to_csv(f"arm_Kp{Kp:.0f}.csv", index=False)

for name in ["Link1", "Link2"]:
    plt.figure()
    for label, table in results.items():
        plt.plot(table["Time"], table[name + " pos"] * 57.3, label=label)
    plt.axhline(0, linestyle="--", color="gray", label="Target")
    plt.title(name + " angle: gain comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint angle (degrees)")
    plt.legend()
    plt.savefig(name + "_comparison.png")
    plt.show()