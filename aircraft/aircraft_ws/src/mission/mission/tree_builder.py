import uuid
import py_trees
from mission import behaviors

def create_mission_tree(node_cfg, ros_node):
    # Recursively parse a YAML dictionary node into a py_trees object

    # Is it a leaf node (an action)?
    if 'action' in node_cfg:
        action = node_cfg['action']
        params = node_cfg.get('params', {})
        name = node_cfg.get('name', f"{action.capitalize()}Action_{uuid.uuid4().hex[:4]}") # Unique fallback in case key 'name' is missing
        if action == 'takeoff':
            return behaviors.TakeoffBehavior(name, ros_node, params)
        elif action == 'land':
            return behaviors.LandBehavior(name, ros_node, params)
        elif action == 'orbit':
            return behaviors.OrbitBehavior(name, ros_node, params)
        elif action == 'wait':
            return behaviors.WaitBehavior(name, ros_node, params)
        elif action == 'offboard':
            return behaviors.OffboardBehavior(name, ros_node, params)
        elif action == 'reposition':
            return behaviors.RepositionBehavior(name, ros_node, params)
        elif action == 'speed':
            return behaviors.SpeedBehavior(name, ros_node, params)
        elif action == 'check_blackboard':
            return behaviors.CheckBlackboardBehavior(name, ros_node, params)
        else:
            ros_node.get_logger().error(f"Unknown action: {action}")
            return py_trees.behaviours.Failure(name=f"Unknown_{action}")

    # Is it a composite node (a sequence or fallback/selector branch)?
    node_type = node_cfg.get('type', 'Sequence')
    name = node_cfg.get('name', f"Unnamed_{node_type}")
    children_cfg = node_cfg.get('children', [])
    if node_type == 'Sequence': # AND logic (all must succeed)
        composite = py_trees.composites.Sequence(name=name, memory=True)
    elif node_type in ['Fallback', 'Selector']: # OR logic (try until one succeeds)
        composite = py_trees.composites.Selector(name=name, memory=True)
    else:
        ros_node.get_logger().error(f"Unknown composite type: {node_type}")
        return py_trees.behaviours.Failure(name=f"Unknown_{node_type}")

    # Recurse on all children
    for child_cfg in children_cfg:
        child_node = create_mission_tree(child_cfg, ros_node)
        composite.add_child(child_node)

    return composite
