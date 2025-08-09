import re
import os
import random
import time
from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
import pybullet_planning as pp

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

# make pp know the client id created by compas_fab client
pp.set_client(client.client_id)
pp.CLIENTS[client.client_id] = True if use_gui else None

planner = PyBulletPlanner(client)

tic = time.time()
with pp.LockRenderer(False):
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
toc = time.time()
print(f"set_robot_cell and set_robot_cell_state took {toc - tic:.3f} seconds")
# wo lock: set_robot_cell and set_robot_cell_state took 29.937 seconds
# with lock: set_robot_cell and set_robot_cell_state took 24.557 seconds

# * Get pyb id for the target bar
target_bar = client.rigid_bodies_puids[active_bar_name][0]
print('target bar name', active_bar_name)
print('target bar id', target_bar)
pp.set_color(target_bar, pp.BLUE)

print("Tool names in robot_cell.tool_models:")
for tool_name, tool_model in robot_cell.tool_models.items():
    print(tool_name)
    pyb_uid = client.tools_puids.get(tool_name, [])
    joints = pp.get_joints(pyb_uid)
    num_joints = len(joints)
    if num_joints > 3:
        # Get the last link index in the kinematic chain
        last_link_index = num_joints - 1
        color = [random.random() for _ in range(3)] + [1.0]  # RGBA
        pp.set_color(pyb_uid, color, link=last_link_index)
    else:
        color = [random.random() for _ in range(3)] + [1.0]  # RGBA
        pp.set_color(pyb_uid, color)

pp.wait_if_gui()