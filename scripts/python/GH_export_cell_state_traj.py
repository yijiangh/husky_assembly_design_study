# export_robotcell_to_json.py

import json
import os
from datetime import datetime
from compas import json_dump, json_load
from compas_fab.robots import RobotCell, RobotCellState


def export_robotcell_and_state(content, filepath, file_prefix=None, trajectory=None):
    os.makedirs(filepath, exist_ok=True)  # Ensure directory exists

    content_class = type(content).__name__
    if file_prefix:
        # If this is a RobotCellState, append timestamp for uniqueness
        if isinstance(content, RobotCellState):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_prefix = f"{file_prefix}_{timestamp}"
        file_name = f"{file_prefix}_{content_class}.json"
    else:
        file_name = f"{content_class}.json"

    full_path = os.path.join(filepath, file_name)

    with open(full_path, "w") as f:
        json_dump(content, f, pretty=True)

    print(f"Exported {content_class} to {full_path}")
    
    # If trajectory is provided and content is RobotCellState, also save the trajectory
    if trajectory is not None and isinstance(content, RobotCellState):
        trajectory_class = type(trajectory).__name__
        if file_prefix:
            trajectory_file_name = f"{file_prefix}_{trajectory_class}.json"
        else:
            trajectory_file_name = f"{trajectory_class}.json"
        
        trajectory_path = os.path.join(filepath, trajectory_file_name)
        
        with open(trajectory_path, "w") as f:
            json_dump(trajectory, f, pretty=True)
        
        print(f"Exported {trajectory_class} to {trajectory_path}")


# Example usage:
# robot_cell, robot_cell_state, trajectory = ... (your objects)
if save_robot_cell and robot_cell is not None:
    export_robotcell_and_state(robot_cell, file_path)
if save_state and robot_cell_state is not None:
    export_robotcell_and_state(
        robot_cell_state,
        os.path.join(file_path, 'RobotCellStates'),
        state_name,
        trajectory  # Pass trajectory if available
    )
