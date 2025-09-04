"""
Grasshopper Python Script: Extract Dual Arm Transformation Data from GraspTarget Tree

This script processes a Grasshopper Tree structure containing GraspTarget objects
and extracts the required transformation data for dual arm planning.

Tree Structure:
- Branch 0: Start poses (2 items: left arm, right arm)
- Branch 1: Goal poses (2 items: left arm, right arm)

Each item is a GraspTarget with the following attributes:
- world_from_bar: Frame representing the bar pose in world coordinates
- bar_from_male_joint: Transformation from bar to male joint
- male_joint_from_tool0: Transformation from male joint to tool0
- world_from_tool0: Computed world to tool0 transformation
"""

import Rhino.Geometry as rg
from compas.geometry import Frame, Transformation

def extract_dual_arm_transformations(grasp_target_tree):
    """
    Extract transformation data from GraspTarget tree for dual arm planning.
    
    Parameters:
    -----------
    grasp_target_tree : Grasshopper.DataTree
        Tree containing GraspTarget objects with start/end and left/right arm data
        
    Returns:
    --------
    tuple : (start_world_from_bar, goal_world_from_bar, left_bar_from_tool0, right_bar_from_tool0)
    """
    
    # Validate tree structure
    if grasp_target_tree.BranchCount < 2:
        raise ValueError(f"Expected at least 2 branches (start/end), got {grasp_target_tree.BranchCount}")
    
    # Extract start poses (Branch 0)
    start_branch = grasp_target_tree.Branch(0)
    if len(start_branch) < 2:
        raise ValueError(f"Expected at least 2 items in start branch (left/right), got {len(start_branch)}")
    
    start_left_target = start_branch[0]  # Left arm start
    start_right_target = start_branch[1]  # Right arm start
    
    # Extract goal poses (Branch 1) 
    goal_branch = grasp_target_tree.Branch(1)
    if len(goal_branch) < 2:
        raise ValueError(f"Expected at least 2 items in goal branch (left/right), got {len(goal_branch)}")
    
    goal_left_target = goal_branch[0]  # Left arm goal
    goal_right_target = goal_branch[1]  # Right arm goal
    
    # Validate that all targets have required attributes
    required_attrs = ['world_from_bar', 'bar_from_male_joint', 'male_joint_from_tool0']
    for target_name, target in [("start_left", start_left_target), ("start_right", start_right_target), 
                               ("goal_left", goal_left_target), ("goal_right", goal_right_target)]:
        for attr in required_attrs:
            if not hasattr(target, attr):
                raise AttributeError(f"{target_name} target missing required attribute: {attr}")
    
    # Extract world_from_bar frames
    # For start: use the left arm's world_from_bar (both arms should have same bar pose)
    start_world_from_bar = start_left_target.world_from_bar
    
    # For goal: use the left arm's world_from_bar (both arms should have same bar pose)
    goal_world_from_bar = goal_left_target.world_from_bar
    
    # Compute bar_from_tool0 transformations for each arm
    # Formula: bar_from_tool0 = bar_from_male_joint * male_joint_from_tool0
    
    # Left arm transformation
    left_bar_from_tool0 = start_left_target.bar_from_male_joint * start_left_target.male_joint_from_tool0
    
    # Right arm transformation  
    right_bar_from_tool0 = start_right_target.bar_from_male_joint * start_right_target.male_joint_from_tool0
    
    # Verify that start and goal have consistent bar poses for each arm
    verbose = getattr(options, 'get', lambda x, y: False)("verbose", False)
    if verbose:
        print("=== Dual Arm Transformation Extraction ===")
        print(f"Start world_from_bar: {start_world_from_bar}")
        print(f"Goal world_from_bar: {goal_world_from_bar}")
        print(f"Left bar_from_tool0: {left_bar_from_tool0}")
        print(f"Right bar_from_tool0: {right_bar_from_tool0}")
        
        # Verify consistency between start and goal
        # Extract translation vectors from transformations
        start_translation = start_world_from_bar.translation_vector
        goal_translation = goal_world_from_bar.translation_vector
        start_goal_bar_diff = (start_translation - goal_translation).length
        print(f"Bar position difference (start to goal): {start_goal_bar_diff:.6f}")
        
        # Verify left/right arm consistency at start
        start_left_translation = start_left_target.world_from_bar.translation_vector
        start_right_translation = start_right_target.world_from_bar.translation_vector
        left_right_start_diff = (start_left_translation - start_right_translation).length
        print(f"Left/Right bar position difference at start: {left_right_start_diff:.6f}")
        
        # Verify left/right arm consistency at goal
        goal_left_translation = goal_left_target.world_from_bar.translation_vector
        goal_right_translation = goal_right_target.world_from_bar.translation_vector
        left_right_goal_diff = (goal_left_translation - goal_right_translation).length
        print(f"Left/Right bar position difference at goal: {left_right_goal_diff:.6f}")
        
        if left_right_start_diff > 0.001 or left_right_goal_diff > 0.001:
            print("WARNING: Left and right arms have different bar poses!")
        else:
            print("✓ Left and right arms have consistent bar poses")
    
    return start_world_from_bar, goal_world_from_bar, left_bar_from_tool0, right_bar_from_tool0

# Main execution
if __name__ == "__main__" or True:  # Always execute in Grasshopper
    # Default options for verbose output
    options = {
        "verbose": True,
        "check_collision": True,
        "tolerance_position": 0.001,
        "tolerance_orientation": 0.01
    }
    
    try:
        # Extract the transformation data
        start_world_from_bar, goal_world_from_bar, left_bar_from_tool0, right_bar_from_tool0 = extract_dual_arm_transformations(grasp_targets)
        
        # These variables are now available for use in the dual arm planning script
        print("Successfully extracted dual arm transformation data!")
        print("Variables available:")
        print("- start_world_from_bar")
        print("- goal_world_from_bar") 
        print("- left_bar_from_tool0")
        print("- right_bar_from_tool0")
        
    except Exception as e:
        print(f"Error extracting dual arm transformations: {str(e)}")
        print("Please check your GraspTarget tree structure:")
        print("- Should have 2 branches (start/end)")
        print("- Each branch should have 2 items (left/right arm)")
        print("- Each GraspTarget should have: world_from_bar, bar_from_male_joint, male_joint_from_tool0")
        raise e
