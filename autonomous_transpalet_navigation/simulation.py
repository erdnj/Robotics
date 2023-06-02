import mujoco
import mujoco_viewer
import numpy
import math

import numpy as np
from scipy.spatial.transform import Rotation
from action import Action
# from dynamic_window_approach import try_test


def get_obstacles(model, wall_number):
    wall_information = []
    obs_info_list = []

    for wall_index in range(1, wall_number + 1):
        wall = model.geom('wall_' + str(wall_index))
        wall_bottom_left_x = (wall.pos[0] - wall.size[0])
        wall_bottom_left_y = (wall.pos[1] - wall.size[1])
        wall_top_right_x = (wall.pos[0] + wall.size[0])
        wall_top_right_y = (wall.pos[1] + wall.size[1])

        wall_corners = [[wall_bottom_left_x, wall_bottom_left_y], [wall_top_right_x, wall_top_right_y]]
        wall_information.append(wall_corners)

    for wall in wall_information:
        wall_bottom_left_x, wall_bottom_left_y = wall[0]
        wall_top_right_x, wall_top_right_y = wall[1]

        for x in np.arange(wall_bottom_left_x, wall_top_right_x, 0.5):
            obs_info_list.append([x, wall_bottom_left_y])
            obs_info_list.append([x, wall_top_right_y])

        for y in np.arange(wall_bottom_left_y, wall_top_right_y, 0.5):
            obs_info_list.append([wall_bottom_left_x, y])
            obs_info_list.append([wall_top_right_x, y])

        obs_info_list.append([wall_top_right_x, wall_top_right_y])

    return obs_info_list, wall_information


def get_obstacles_with_radius(model, wall_number):
    wall_information_list = []
    obs_info = []

    for wall_index in range(1, wall_number + 1):
        wall = model.geom('wall_' + str(wall_index))
        wall_bottom_left_x = (wall.pos[0] - wall.size[0])
        wall_bottom_left_y = (wall.pos[1] - wall.size[1])
        wall_top_right_x = (wall.pos[0] + wall.size[0])
        wall_top_right_y = (wall.pos[1] + wall.size[1])

        wall_corners = [[wall_bottom_left_x, wall_bottom_left_y], [wall_top_right_x, wall_top_right_y]]
        wall_information_list.append(wall_corners)

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
            wall_radius = wall.size[0] / 2
            start_y = (wall.pos[1] - wall.size[1]) + wall_radius
            x = wall.pos[0]
            end_y = (wall.pos[1] + wall.size[1]) - wall_radius
            for i in np.arange(start_y, end_y, 0.1):
                obs_info.append((x, i, wall_radius))

    return obs_info, wall_information_list


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

# obs_info_list, _ = get_obstacles(model, 12)
# obs_info_array = np.array(obs_info_list)
# try_test(obs_info_array)

with_dynamic = True

dynamic_object_actuators = [data.actuator("dynamic_object_1"), data.actuator("dynamic_object_2"), data.actuator("dynamic_object_3")]
dynamic_object_positions = [data.site("dynamic_object_1_link"), data.site("dynamic_object_2_link"), data.site("dynamic_object_3_link")]
dynamic_object_speed = 0.2
dynamic_object_num = len(dynamic_object_positions)

obs_info_list, _ = get_obstacles_with_radius(model, 12)


for _ in range(dynamic_object_num):
    # obs_info_list.append([0.0, 0.0])
    obs_info_list.append((20.0, 20.0, 0.2))

obs_info_array = np.array(obs_info_list)


def update_dynamic_objects(step_counter):
    for dynamic_object_index in range(0, len(dynamic_object_actuators)):
        dynamic_object_actuators[dynamic_object_index].ctrl = abs(math.sin(dynamic_object_speed * math.pi * step_counter / 1000)) * 100
        # print(dynamic_object_positions[dynamic_object_index].xpos)
        #obs_info_array[-dynamic_object_num:] = [link.xpos[0:2] for link in dynamic_object_positions]
        obs_info_array[-dynamic_object_num:,0:2] = [ link.xpos[0:2] for link in dynamic_object_positions]


mujoco.mj_step(model, data)

actioner = Action(data, obs_info_array, 1, dynamic_object_num, with_dynamic=with_dynamic)
print(dynamic_object_num)

step_counter = 0
for _ in range(40000):
    previous_time = data.time

    while (data.time - previous_time) < 1.0 / 60:
        if with_dynamic:
            update_dynamic_objects(step_counter)
        step_counter = step_counter + 0.2
        mujoco.mj_step(model, data)

    if viewer.is_alive:
        result = actioner.forward()
        if result == -1:
            print("break")
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
    else:
        break

viewer.close()






