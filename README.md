# 2-Link Arm PD Control in MuJoCo

A planar 2-link robot arm simulated in MuJoCo and controlled by a joint-space PD controller written in Python. Joint angles, velocities and torques are logged every step and analysed with Pandas and Matplotlib.

## Files
- `arm.xml`: arm model (two links, two hinge joints about the y axis, two motors, gravity, 2 ms timestep)
- `arm_pd_control.py`: simulation, PD controller, metrics and plots
- `Link1_comparison.png`, `Link2_comparison.png`: joint angle against time for each gain set

## Run
```
pip install mujoco pandas matplotlib
python arm_pd_control.py
```

## Controller
`torque = Kp * (target - angle) - Kd * velocity`, computed once per 2 ms step for each joint. The target is 0 for both joints (arm held horizontal).

## Results (5 s simulation, target 0 deg)

| Kp | Kd | Link 1 steady-state error | Link 1 overshoot | Link 2 steady-state error | Link 2 overshoot |
|----|----|----|----|----|----|
| 50 | 5 | 11.01 deg | 2.65 deg (24.1 %) | 2.73 deg | 0.86 deg (31.6 %) |
| 100 | 10 | 5.59 deg | 0.57 deg (10.2 %) | 1.39 deg | 0.20 deg (14.2 %) |
| 200 | 10 | 2.81 deg | 0.66 deg (23.5 %) | 0.70 deg | 0.22 deg (30.7 %) |

Kp = 200, Kd = 20 was unstable: the arm oscillated and did not settle.

## Observations
- With a constant target, the arm settles below it, because holding the arm up against gravity needs a nonzero error. The steady-state error roughly halves each time Kp doubles (11.01, 5.59, 2.81 deg for Link 1), consistent with error = gravity torque / Kp.
- Overshoot was lowest for Kp = 100, Kd = 10. Raising Kp to 200 while keeping Kd = 10 gave back more overshoot, because damping did not increase along with stiffness.
- The instability at Kd = 20 is probably related to the discrete 2 ms control step, but I have not confirmed this.
- Both joints peak at the same time, because the shoulder's motion disturbs the elbow (the two links are coupled).

## Not done
Forward-kinematics validation against MuJoCo, a moving target trajectory, and an integral term to remove the steady-state error.