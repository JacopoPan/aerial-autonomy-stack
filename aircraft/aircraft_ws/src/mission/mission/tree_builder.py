import uuid
import py_trees
from mission import behaviors

ACTIONS = { # YAML 'action' -> behavior
    'takeoff': behaviors.TakeoffBehavior,
    'land': behaviors.LandBehavior,
    'orbit': behaviors.OrbitBehavior,
    'wait': behaviors.WaitBehavior,
    'offboard': behaviors.OffboardBehavior,
    'reposition': behaviors.RepositionBehavior,
    'speed': behaviors.SpeedBehavior,
    'check_blackboard': behaviors.CheckBlackboardBehavior,
}

COMPOSITES = { # YAML 'type' -> composite
    'Sequence': py_trees.composites.Sequence, # AND logic (all must succeed)
    'Selector': py_trees.composites.Selector, # OR logic (try until one succeeds)
    'Fallback': py_trees.composites.Selector, # Alias of Selector
}

def create_mission_tree(node_cfg, ros_node):
    # Recursively parse a YAML dictionary node into a py_trees object

    # Is it a leaf node (an action)?
    if 'action' in node_cfg:
        action = node_cfg['action']
        name = node_cfg.get('name', f"{action.capitalize()}Action_{uuid.uuid4().hex[:4]}") # Unique fallback in case key 'name' is missing
        behavior_class = ACTIONS.get(action)
        if behavior_class is None:
            raise ValueError(f"Unknown action: {action}") # Fail loudly on malformed YAML mission
        node = behavior_class(name, ros_node, node_cfg.get('params', {}))

    # Or is it a composite node (a sequence or fallback/selector branch)?
    else:
        node_type = node_cfg.get('type', 'Sequence')
        name = node_cfg.get('name', f"Unnamed_{node_type}")
        composite_class = COMPOSITES.get(node_type)
        if composite_class is None:
            raise ValueError(f"Unknown composite type: {node_type}") # Fail loudly on malformed YAML mission
        node = composite_class(name=name, memory=node_cfg.get('memory', True)) # Default memory to True

        # Recurse on all children
        for child_cfg in node_cfg.get('children', []):
            node.add_child(create_mission_tree(child_cfg, ros_node))

    # Wrap the node in a Repeat decorator if the YAML sets 'repeat' (a value <= 0 loops forever)
    count = int(node_cfg.get('repeat', 1)) # Cast to integer, default to 1 repetition if the key is absent, fail loudly if not a number
    if count == 1:
        return node
    return py_trees.decorators.Repeat(name=f"Repeat{count}x_{node.name}", child=node, num_success=count)
