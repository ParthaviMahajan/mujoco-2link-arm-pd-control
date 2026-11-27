import mujoco
import numpy
import math

model = mujoco.MjModel.from_xml_path("arm.xml")
data = mujoco.MjData(model)

angles = [(0,0),(90,0),(-90,0),(180,0),(45,-45),(20,-50),(-45,120),(135,-70),(10,170),(-100,-40),(60,60),(37,112),(-63,28)]

def calculate(theta1, theta2):
     coordinates = [0.5*(math.cos(theta1)+math.cos(theta1+theta2)), 1-0.5*(math.sin(theta1)+math.sin(theta1+theta2))]
     return coordinates

max_x = 0
max_z = 0

for theta1, theta2 in angles:
    data.qpos[0] = numpy.deg2rad(theta1)
    data.qpos[1] = numpy.deg2rad(theta2)
    calculated = calculate(data.qpos[0], data.qpos[1])
    mujoco.mj_forward(model, data)
    position = data.site("tip").xpos
    max_x = max(max_x, abs(calculated[0]-position[0]))
    max_z = max(max_z, abs(calculated[1]-position[2]))
    print("Angle in degree:", theta1, theta2,)
    print("Calculated:", calculated[0], calculated[1])
    print("MuJoCo:", position[0], position[2])
    print("Error:", calculated[0]-position[0], calculated[1]-position[2])
    
print("Max error:", max_x, max_z)
