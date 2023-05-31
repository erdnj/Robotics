import numpy as np
from enum import Enum

from rrt_star import get_rrt_star_path

States = Enum('State', ['Lift_Up', 'Lift_Down', 'Fork_Follow', 'Head_Follow', 'Head_Turn', 'Fork_Adjust', 'Fork_Turn',
                        'Fork_Step_In', 'Fork_Step_Out', 'Path_Plan', 'End'])


class Action:
    def __init__(self, data, wall_info_list, num_task):
        self.drive_speed_actuator = data.actuator("drive_speed")
        self.steering_angle_actuator = data.actuator("steering_angle")
        self.fork_height_actuator = data.actuator("fork_height")

        self.robot_link = data.site("base_link")
        self.front_robot_link = data.site("front_base_link")
        self.palet_link = None
        self.drop_link = None
        self.wall_list = wall_info_list
        self.task_list = [(data.site("palet_link{}".format(i)),
                           data.site("palet_drop{}".format(i))) for i in range(1, num_task + 1)]
        self.currentTaskId = 0
        self.state = States.Path_Plan
        self.isWeighted = False
        self.currentPath = None
        self.currentLine = None

        # path planning vars
        self.reference_path = None

        # goal related
        self.goal_cross_line = None
        self.goal_point = None
        self.goal_ori = None
        self.stepInLine = None

        # Steer Logic Related Vars
        self.maxNextPathId = 0
        self.currentPathId = 0
        self.currentLine = None
        self.nextLine = None
        self.referencedNextLine = None

        # stop lines
        self.end_cross_line = None
        self.goal_cross_line = None

        # steer parameters
        self.par_next_dist = -0.2  # 0.0001
        self.par_line = None
        self.par_direction = None
        self.par_line_fork = 4.0
        self.par_direction_fork = 5.0
        self.par_line_head = -1.5
        self.par_direction_head = -1.0

    def forward(self):
        match self.state:
            case States.Lift_Up:
                self.lift_up()
            case States.Lift_Down:
                self.lift_down()
            case States.Fork_Follow:
                self.fork_follow()
            case States.Head_Follow:
                self.head_follow()
            case States.Head_Turn:
                self.head_turn()
            case States.Fork_Adjust:
                self.fork_adjust()
            case States.Fork_Turn:
                self.fork_turn()
            case States.Fork_Step_In:
                self.fork_step_in()
            case States.Fork_Step_Out:
                self.fork_step_out()
            case States.Path_Plan:
                self.find_path()
            case States.End:
                return self.end()

    def find_path(self, extra_obs_list=None):
        goal_link = self.task_list[self.currentTaskId][1] if self.isWeighted else self.task_list[self.currentTaskId][0]

        obs_list = self.wall_list
        if extra_obs_list is not None:
            np.concatenate((obs_list, extra_obs_list), axis=0)

        start_point = self.front_robot_link.xpos[0:2] if self.isWeighted else self.robot_link.xpos[0:2]
        start_ori = - self.front_robot_link.xmat[[0, 3]] if self.isWeighted else self.front_robot_link.xmat[[0, 3]]

        self.goal_point, self.goal_ori = get_goal_point(goal_link)
        reference_path_2column = get_rrt_star_path(start_point, self.goal_point, self.wall_list)
        self.reference_path = np.pad(reference_path_2column, ((0, 0), (0, 1)), 'constant', constant_values=(0, 0))
        print(self.reference_path)
        # after found path set reference_path
        # Steer Logic Related Vars
        self.end_cross_line = None
        self.maxNextPathId = self.reference_path.shape[0] - 2
        self.currentPathId = 0
        self.currentLine = line_formula(self.reference_path[0], self.reference_path[1])
        if len(self.reference_path) > 2:
            self.nextLine = line_formula(self.reference_path[1], self.reference_path[2])
            self.referencedNextLine = referenced_line(self.reference_path[0], self.nextLine[0])
        self.state = States.Head_Follow if self.isWeighted else States.Fork_Follow

    def path_follow(self):
        cur_pos = self.front_robot_link.xpos[0:2] if self.state == States.Head_Follow \
            else self.robot_link.xpos[0:2]
        direction = - self.front_robot_link.xmat[[0, 3]] if self.state == States.Head_Follow \
            else self.front_robot_link.xmat[[0, 3]]

        # dotProduct = np.dot(direction, currentLine[1])
        cross_product = np.cross(direction, self.currentLine[1])

        dist_to_line = line_distance(cur_pos=cur_pos, current_line=self.currentLine[0])
        new_steer = dist_to_line * self.par_line + cross_product * self.par_direction
        self.steering_angle_actuator.ctrl = np.clip(new_steer, -1, 1)

        # transfer logic for next line
        if self.nextLine is not None:  # if there is a nextline
            if line_distance(cur_pos=cur_pos, current_line=self.referencedNextLine) > self.par_next_dist:
                self.currentPathId += 1
                self.currentLine = self.nextLine
                if self.currentPathId != self.maxNextPathId:
                    self.nextLine = line_formula(self.reference_path[self.currentPathId + 1],
                                                 self.reference_path[self.currentPathId + 2])
                    self.referencedNextLine = referenced_line(self.reference_path[self.currentPathId], self.nextLine[0])
                else:
                    self.nextLine = None
        else:
            if self.end_cross_line is None:
                point = self.reference_path[self.currentPathId + 1]
                direction_line = point - self.reference_path[self.currentPathId]
                direction = direction_line / np.linalg.norm(direction_line)
                self.end_cross_line = line_formula_from_direction(point, direction)

            distance = line_distance(cur_pos, self.end_cross_line)
            if distance > -0.0001:
                self.drive_speed_actuator.ctrl = 0
                self.state = States.Fork_Adjust
            elif distance > -0.4:
                self.drive_speed_actuator.ctrl = 0.1 if self.state == States.Head_Follow else -0.1

    def fork_follow(self):
        self.par_line = self.par_line_fork
        self.par_direction = self.par_direction_fork
        self.drive_speed_actuator.ctrl = -1
        self.path_follow()

    def head_follow(self):
        self.par_line = self.par_line_head
        self.par_direction = self.par_direction_head
        self.drive_speed_actuator.ctrl = 1
        self.path_follow()

    def head_turn(self):
        None

    def fork_adjust(self):
        self.steering_angle_actuator.ctrl = -1

        direction = self.robot_link.xmat[[0, 3]]
        cross_product = np.cross(direction, self.goal_ori)
        self.drive_speed_actuator.ctrl = np.clip(cross_product * 10, -1, 1)
        print(cross_product)
        if np.abs(cross_product) < 0.0001:
            self.drive_speed_actuator.ctrl = 0
            self.steering_angle_actuator.ctrl = 0
            self.state = States.Fork_Step_In

    def fork_turn(self):
        None

    def fork_step_in(self):
        if self.goal_cross_line is None:
            goal_link = self.task_list[self.currentTaskId][1] if self.isWeighted else \
                self.task_list[self.currentTaskId][0]
            target_point = goal_link.xpos[0:2]
            self.goal_cross_line = line_formula_from_direction(target_point, -self.goal_ori)
            self.drive_speed_actuator.ctrl = -1
            self.steering_angle_actuator.ctrl = 0
            self.stepInLine = line_formula(self.goal_point, target_point)

        cur_pos = self.robot_link.xpos[0:2]
        direction = self.front_robot_link.xmat[[0, 3]]
        cross_product = np.cross(direction, self.stepInLine[1])

        dist_to_line = line_distance(cur_pos=cur_pos, current_line=self.stepInLine[0])
        new_steer = dist_to_line * self.par_line_fork + cross_product * self.par_direction_fork
        self.steering_angle_actuator.ctrl = np.clip(new_steer, -1, 1)

        distance = line_distance(cur_pos, self.goal_cross_line)

        if distance < -0.15:
            self.drive_speed_actuator.ctrl = 0
            self.goal_cross_line = None
            self.state = States.Lift_Down if self.isWeighted else States.Lift_Up

    def lift_up(self):
        if self.fork_height_actuator.ctrl < 1:
            self.fork_height_actuator.ctrl += 0.05
        else:
            self.isWeighted = True
            self.state = States.Path_Plan

    def lift_down(self):
        if self.fork_height_actuator.ctrl > 0:
            self.fork_height_actuator.ctrl -= 0.05
        else:
            self.isWeighted = False
            self.state = States.Fork_Step_Out

    def fork_step_out(self):
        if self.goal_cross_line is None:
            goal_link = self.task_list[self.currentTaskId][0]
            self.goal_cross_line = line_formula_from_direction(goal_link.xpos[0:2], -self.goal_ori)
            self.drive_speed_actuator.ctrl = 1
            self.steering_angle_actuator.ctrl = 0

        cur_pos = self.robot_link.xpos[0:2]
        distance = line_distance(cur_pos, self.goal_cross_line)
        if distance > 0.8:
            self.drive_speed_actuator.ctrl = 0
            self.goal_cross_line = None
            if self.currentTaskId + 1 == len(self.task_list):
                self.state = States.End
            else:
                self.currentTaskId += 1
                self.state = States.Path_Plan

    @staticmethod
    def end():
        print("End of simulation")
        return -1


# helper funcs
def get_goal_point(goal_link):
    pos = goal_link.xpos[0:2]
    direction = goal_link.xmat[[0, 3]]
    unit_direction = direction / np.linalg.norm(direction)
    return pos + unit_direction * 1.2, -direction


def line_formula(from_point, to_point):
    vector = np.array([to_point[0] - from_point[0], to_point[1] - from_point[1]])
    return np.array([
        from_point[1] - to_point[1],  # y0 - y1
        to_point[0] - from_point[0],  # x1 - x0
        from_point[0] * to_point[1],  # x0*y1
        - from_point[1] * to_point[0]  # x1*y0
    ]), vector / np.linalg.norm(vector)


def line_formula_from_direction(point, direction):
    return np.array([
        direction[0],  # xd
        direction[1],  # yd
        - point[0] * direction[0],  # -xd*x0
        - point[1] * direction[1]  # -yd*y0
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
