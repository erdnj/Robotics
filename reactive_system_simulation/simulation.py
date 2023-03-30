import mujoco
import mujoco_viewer
import numpy as np
from matplotlib import pyplot as plt


# get line formula as a matrix
def line_formula(from_point, to_point):
    return np.array([
        from_point[1] - to_point[1],  # y0 - y1
        to_point[0] - from_point[0],  # x1 - x0
        from_point[0] * to_point[1],  # x0*y1
        - from_point[1] * to_point[0]  # x1*y0
    ])


# got distance a point from a line
def line_distance(cur_pos, current_line):
    padded_pos = np.pad(cur_pos[0:2], (0, 2), constant_values=(1, 1)).reshape(4, 1)
    distance = np.matmul(current_line, padded_pos)[0]
    return distance


# To got positive distance after surpass the next line
def referenced_line(reference_pos, current_line):
    distance = line_distance(reference_pos, current_line)
    if distance < 0:
        return current_line
    else:
        return -current_line


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

translation_matrix = np.asarray(x_dir)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data)

data.ctrl[0] = 0.0  # steering
data.ctrl[1] = 1.0  # throttle, power to an engin
# simulate and render + the use of markers

car = data.body('buddy')
car_id = car.id

# Lists to keep outputs
timevals = []
positions = []
velocities = []
accelerations = []
gyros = []
inputs = []

# our reference path
reference_path_raw = [[0, 0, 0], [8.7, 0, 0], [8.7, -8, 0], [17.3, -8, 0]]
reference_path = np.asarray(reference_path_raw)

# Steer Logic Related Vars
maxNextPathId = reference_path.shape[0] - 2
currentPathId = 0
currentLine = line_formula(reference_path[0], reference_path[1])
nextLine = line_formula(reference_path[1], reference_path[2])
referencedNextLine = referenced_line(reference_path[0], nextLine)

# steer parameters
par_next_dist = 0.0001
par_steer = -1.3

for frame in range(14000):
    # data.ctrl[0] = in_inputs[frame][0]
    # data.ctrl[1] = in_inputs[frame][1]
    # direction = data.xmat[car_id].reshape(3, 3)[[0, 1], 0]

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
            mat=np.matmul(data.xmat[car_id].reshape(3, 3), translation_matrix),
            size=[0.01, 0.01, 2],
            rgba=[1, 0, 0, 1],
            type=mujoco.mjtGeom.mjGEOM_ARROW,
            label="")
        for i in range(1, reference_path.shape[0]):
            sizeArray = np.absolute((reference_path[i] - reference_path[i - 1]) / 2.0)
            sizeArray[sizeArray == 0] = 0.01
            viewer.add_marker(
                pos=(reference_path[i - 1] + reference_path[i]) / 2.0,
                size=sizeArray,
                rgba=[1, 1, 0, 0.7],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                label="track"
            )
        viewer.render()

    else:
        break

    # Steer Logic Region
    newSteer = line_distance(cur_pos=data.xpos[car_id], current_line=currentLine) * par_steer
    data.ctrl[0] = newSteer

    # transfer logic to next line
    if nextLine is not None:  # if there is a nextline
        if line_distance(cur_pos=data.xpos[car_id], current_line=referencedNextLine) > par_next_dist:
            currentPathId += 1
            currentLine = nextLine
            if currentPathId != maxNextPathId:
                nextLine = line_formula(reference_path[currentPathId + 1], reference_path[currentPathId + 2])
                referencedNextLine = referenced_line(reference_path[currentPathId], nextLine)
            else:
                nextLine = None
    # /Steer Logic

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
height = 4800
figsize = (width / dpi, height / dpi)
_, ax = plt.subplots(5, 1, figsize=figsize, dpi=dpi, sharex=True)

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

positions_np = np.asarray(positions)
ax[3].set_xticks(range(-15, 15))
ax[3].set_yticks(range(-15, 15))
ax[3].plot(*reference_path.transpose()[0:2], "-", color="green", linewidth=3)
ax[3].plot(*positions_np.transpose(), "-", color="red")
ax[3].set_xlabel('x')
ax[3].set_ylabel('y')
_ = ax[3].set_title('X-Y position')

ax[4].plot(timevals, gyros)
ax[4].set_xlabel('time (seconds)')
ax[4].set_ylabel('gyro')
_ = ax[4].set_title('Gyro')

plt.show()

# close
viewer.close()
