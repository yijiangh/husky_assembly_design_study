import re
import os
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner

HERE = os.path.dirname(os.path.abspath(__file__))
problem_name = '250806_RobotX_box_redo'

robot_cell = json_load(os.path.join(HERE, '..', problem_name, 'RobotCell.json'))
state_file = 'robotx_box_A5-S4_end_RobotCellState.json'
# state_file = 'robotx_box_A6-S4_end_RobotCellState.json'

match = re.search(r'_A(\d+)-', state_file)
active_bar_name = f"b{match.group(1)}_0" if match else None

robot_cell_state = json_load(os.path.join(HERE, '..', problem_name, 'RobotCellStates', state_file))

use_gui = True
client = PyBulletClient(connection_type="gui" if use_gui else "direct", verbose=True)
client.__enter__()

planner = PyBulletPlanner(client)
planner.set_robot_cell(robot_cell)
planner.set_robot_cell_state(robot_cell_state)

target_bar = client.rigid_bodies_puids[active_bar_name][0]


print('target bar name', active_bar_name)
print('target bar id', target_bar)

# import pybullet_planning as pp
# pp.set_color(target_bar, [0, 0, 1])

import pybullet as p
p.changeVisualShape(target_bar, -1, shapeIndex=-1, rgbaColor=(0, 0, 1, 1), physicsClientId=client.client_id)

input('Press Enter to continue...')