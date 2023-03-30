# import keyword
import mujoco
import mujoco_viewer
# import numpy as np
# from numpy import savetxt
from numpy import loadtxt
from matplotlib import pyplot as plt


import keyboard

model = mujoco.MjModel.from_xml_path("../models/race_area/scene.xml")
data = mujoco.MjData(model)

x_dir = [[0, 0, 1],
         [0, 1, 0],
         [-1, 0, 0]]

y_dir = [[1, 0, 0],
         [0, 0, 1],
         [0, -1, 0]]

z_dir = [[1, 0, 0],
         [0, 1, 0],
         [0, 0, 1]]

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

data.ctrl[0] = 0.0  # steering
data.ctrl[1] = 1.0  # throttle, power to an engin
# simulate and render + the use of markers

car = data.body('buddy')
car_id = car.id

timevals = []
positions = []
velocities = []
accelerations = []
gyros = []
inputs = []


def inc_little(array, index, add):
    array[index] += add


# keyboard.on_press_key('p', lambda _: inc_little(data.ctrl, 1, 0.1))
# keyboard.on_press_key('ş', lambda _: inc_little(data.ctrl, 1, -0.1))
# keyboard.on_press_key('i', lambda _: inc_little(data.ctrl, 0, -0.05))
# keyboard.on_press_key('l', lambda _: inc_little(data.ctrl, 0, 0.05))

in_inputs = loadtxt('inputs.csv', delimiter=',')


for frame in range(8000):
    data.ctrl[0] = in_inputs[frame][0]
    data.ctrl[1] = in_inputs[frame][1]
    if viewer.is_alive:
        mujoco.mj_step(model, data)

        viewer.add_marker(
            pos=[0, 0, 0],
            size=[0.05, 0.05, 0.05],
            rgba=[1, 1, 1, 1],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            label="origin")
        viewer.add_marker(
            pos=data.xpos[car_id],
            mat=x_dir,
            size=[0.01, 0.01, 2],
            rgba=[1, 0, 0, 1],
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            label="")
        viewer.render()

    else:
        break
    timevals.append(data.time)
    positions.append(car.xpos[0:2].copy())
    velocities.append(data.sensor("velocimeter").data.copy())
    accelerations.append(data.sensor("accelerometer").data.copy())
    gyros.append(data.sensor("gyro").data.copy())
    inputs.append(data.ctrl.copy())

# ninputs = np.asarray(inputs);
# savetxt('inputs.csv', ninputs, delimiter=',')

dpi = 300
width = 1800
height = 2400
figsize = (width / dpi, height / dpi)
_, ax = plt.subplots(4, 1, figsize=figsize, dpi=dpi, sharex=True)

ax[0].plot(timevals, velocities)
# ax[0].set_xlabel('time (seconds)')
ax[0].set_ylabel('radians / second')
_ = ax[0].set_title('Velocity')

ax[1].plot(timevals, accelerations)
# ax[1].set_xlabel('time (seconds)')
ax[1].set_ylabel('Acc')
_ = ax[1].set_title('Acceleration')

ax[2].plot(timevals, positions)
# ax[2].set_xlabel('time (seconds)')
ax[2].set_ylabel('position')
_ = ax[2].set_title('Position')

ax[3].plot(timevals, gyros)
ax[3].set_xlabel('time (seconds)')
ax[3].set_ylabel('gyro')
_ = ax[3].set_title('Gyro')

plt.show()

# close
viewer.close()
