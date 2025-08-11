# Husky Assembly Design Study Data

This repository contains design data, planned trajectories, and some simple data visualization tools for the Husky Assembly Project. 
The data is primarily stored as JSON files and can be processed using the provided Python scripts.

## 📊 Data Formats

### RobotCell.json
Contains the robot cell configuration including:
- Robot definitions and kinematics
- Tool definitions and attachments
- Workspace configurations
- Planning parameters

### RobotCellState.json
Contains the current state of the robot cell including:
- Joint configurations
- Tool attachments
- Object poses
- Planning goals

### URDF Files
- **PointTool.urdf**: Validation point tool with collision and visual meshes
- **BoardTool.urdf**: Validation board tool with collision and visual meshes

## 🛠️ Keyframe Authoring

The `scripts/keyframe_gh_interface` directory contains Grasshopper scripts for:
- **Keyframe creation**: Authoring assembly keyframes (by manually assigning assembly sequence and tasks, placing robot base, selecting grasps)

## 📊 Data Visualization

The `scripts/python` directory contains python scripts for de-serializing and visualization of the data.

## 🔄 Workflow Integration

This data repository integrates with:

1. **[husky_assembly_tamp](https://github.com/yijiangh/husky_assembly_tamp)**: Task and Motion Planning framework
2. **[husky-assembly-teleop](https://github.com/yijiangh/husky-assembly-teleop)**: Hardware deployment and execution

### Typical Workflow:
1. **Design**: Create assembly designs in Grasshopper
2. **Keyframe**: Author keyframes in Grasshopper
3. **Plan**: Generate trajectories using TAMP framework
4. **Deploy**: Execute on hardware using teleop system