import mujoco
import mujoco_viewer
import numpy
import math

import numpy as np
from scipy.spatial.transform import Rotation
from action import Action

model = mujoco.MjModel.from_xml_path('../models/atlas_transpalet/scene.xml')
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)
state = 0
state_end_time = 0

drive_speed_actuator = data.actuator("drive_speed")
steering_angle_actuator = data.actuator("steering_angle")
fork_height_actuator = data.actuator("fork_height")

robot_base_link = data.site("base_link")
palet_link = data.site("palet_link1")
palet_drop = data.site("palet_drop1")

wall_number = 12
wall_information = []
obs_info = []

for wall_index in range(1, wall_number + 1):
    wall = model.geom('wall_' + str(wall_index))
    wall_bottom_left_x = (wall.pos[0] - wall.size[0])
    wall_bottom_left_y = (wall.pos[1] - wall.size[1])
    wall_top_right_x = (wall.pos[0] + wall.size[0])
    wall_top_right_y = (wall.pos[1] + wall.size[1])

    wall_corners = [[wall_bottom_left_x, wall_bottom_left_y], [wall_top_right_x, wall_top_right_y]]
    wall_information.append(wall_corners)

for wall_index in range(1, wall_number + 1):
    wall = model.geom('wall_' + str(wall_index))
    if wall.size[0] > wall.size[1]:
        wall_radius = wall.size[1] / 2
        start_x = (wall.pos[0] - wall.size[0]) + wall_radius
        y = wall.pos[1]
        end_x = (wall.pos[0] + wall.size[0]) - wall_radius
        for i in np.arange(start_x, end_x, 0.1):
            obs_info.append((i, y, wall_radius))
    else:
        wall_radius = wall.size[0]/2
        start_y = (wall.pos[1] - wall.size[1]) + wall_radius
        x = wall.pos[0]
        end_y = (wall.pos[1] + wall.size[1]) - wall_radius
        for i in np.arange(start_y, end_y, 0.1):
            obs_info.append((x, i, wall_radius))
mujoco.mj_step(model, data)

actioner = Action(data, obs_info, 1)

for _ in range(40000):
    previous_time = data.time

    while (data.time - previous_time) < 1.0 / 60:
        mujoco.mj_step(model, data)

    if viewer.is_alive:
        result = actioner.forward()
        if result == -1:
            break
        if actioner.reference_path is not None:
            reference_path = actioner.reference_path
            for i in range(1, reference_path.shape[0]):
                path_vector = (reference_path[i] - reference_path[i - 1])
                path_step_vector = path_vector * 0.2 / np.linalg.norm(path_vector)
                current_dot = reference_path[i - 1]
                step_size = np.linalg.norm(path_vector) / 0.2
                for _ in range(round(step_size)):
                    viewer.add_marker(
                        pos=current_dot,
                        size=[0.02, 0.02, 0.01],
                        rgba=[1, 1, 0, 0.7],
                        type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    )
                    current_dot = current_dot + path_step_vector
        viewer.add_marker(
            pos=[*data.site("base_link").xpos[0:2],0],
            size=[0.02, 0.02, 0.01],
            rgba=[1, 0, 0, 0.7],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
        )
        viewer.render()
        # if state == 0:
        #     forklift_rotation_matrix = numpy.array(robot_base_link.xmat).reshape(3, 3)
        #     forklift_rotation = Rotation.from_matrix(forklift_rotation_matrix)
        #     forklift_rotation_angles = forklift_rotation.as_euler("zyx", degrees=True)
        #
        #     if abs(forklift_rotation_angles[0] + 90) < 1:
        #         drive_speed_actuator.ctrl = 0.0
        #         steering_angle_actuator.ctrl = 0.0
        #         state = state + 1
        #         state_end_time = data.time
        #     else:
        #         drive_speed_actuator.ctrl = 1.0
        #         steering_angle_actuator.ctrl = -1.0
        #
        # elif state == 1:
        #     drive_speed_actuator.ctrl = 0.0
        #     steering_angle_actuator.ctrl = 0.0
        #
        #     if abs(state_end_time - data.time) > 2:
        #         state = state + 1
        #         state_end_time = data.time
        #
        # elif state == 2:
        #     forklift_position = robot_base_link.xpos
        #     palet_position = palet_link.xpos
        #
        #     if abs(forklift_position[1] - palet_position[1]) < 0.01:
        #         drive_speed_actuator.ctrl = 0.0
        #         steering_angle_actuator.ctrl = 0.0
        #         state = state + 1
        #     else:
        #         drive_speed_actuator.ctrl = -1 * abs(forklift_position[1] - palet_position[1])
        #         steering_angle_actuator.ctrl = 0.0
        #
        # elif state == 3:
        #     forklift_rotation_matrix = numpy.array(robot_base_link.xmat).reshape(3, 3)
        #     forklift_rotation = Rotation.from_matrix(forklift_rotation_matrix)
        #     forklift_rotation_angles = forklift_rotation.as_euler("zyx", degrees=True)
        #     forklift_position = robot_base_link.xpos
        #
        #     if abs(forklift_rotation_angles[0] - 0) < 1:
        #         drive_speed_actuator.ctrl = 0.0
        #         steering_angle_actuator.ctrl = 0.0
        #         state = state + 1
        #         state_end_time = data.time
        #     else:
        #         drive_speed_actuator.ctrl = 1.0
        #         steering_angle_actuator.ctrl = 1.0
        #
        # elif state == 4:
        #     forklift_position = robot_base_link.xpos
        #     palet_position = palet_link.xpos
        #
        #     if abs(forklift_position[0] - (palet_position[0] - 0.25)) < 0.1:
        #         drive_speed_actuator.ctrl = 0.0
        #         steering_angle_actuator.ctrl = 0.0
        #         state = state + 1
        #         state_end_time = data.time
        #
        #     else:
        #         drive_speed_actuator.ctrl = (-1 * abs(forklift_position[0] - (palet_position[0] - 0.25)) / 2)
        #         steering_angle_actuator.ctrl = 0.0
        #
        # elif state == 5:
        #     drive_speed_actuator.ctrl = 0.0
        #     steering_angle_actuator.ctrl = 0.0
        #
        #     if abs(state_end_time - data.time) > 2:
        #         state = state + 1
        #         state_end_time = data.time
        #     else:
        #         fork_height_actuator.ctrl = abs(state_end_time - data.time)
        #
        # elif state == 6:
        #     drive_speed_actuator.ctrl = 0.60
        #     steering_angle_actuator.ctrl = 0.8
        #     fork_height_actuator.ctrl = 1.0
        #
        # viewer.render()
        # print(state)
    else:
        break

viewer.close()
