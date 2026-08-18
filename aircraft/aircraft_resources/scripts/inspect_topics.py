"""
Inspect all topics' message type, publisher node(s), and subscriber nodes

Use as:
    python3 /aas/aircraft_resources/scripts/inspect_topics.py
"""
import subprocess

def inspect_topics():
    try:
        topics = subprocess.check_output(['ros2', 'topic', 'list']).decode().split()
    except Exception:
        print("Failed to run 'ros2 topic list'. Did you source your ROS setup.bash?")
        return

    print(f"{'TOPIC':<60} | {'TYPE':<40} | {'PUBLISHERS (Nodes)':<40} | {'SUBSCRIBERS (Nodes)':<40}")
    print("-" * 185)

    for t in topics:
        try:
            info = subprocess.check_output(['ros2', 'topic', 'info', '-v', t], stderr=subprocess.DEVNULL).decode().splitlines()
        except Exception:
            continue

        msg_type = "Unknown"
        pubs, subs = [], []
        current_section = None

        for line in info:
            line = line.strip()
            if line.startswith("Type:"):
                msg_type = line.split("Type:")[1].strip()
            elif line.startswith("Publisher count:"):
                current_section = "pub"
            elif line.startswith("Subscription count:"):
                current_section = "sub"
            elif line.startswith("Node name:"):
                node_name = line.split("Node name:")[1].strip()
                if node_name != "UNKNOWN" and not node_name.startswith("_ros2cli"):
                    if current_section == "pub":
                        pubs.append(node_name)
                    elif current_section == "sub":
                        subs.append(node_name)

        pub_str = ", ".join(pubs) if pubs else "None"
        sub_str = ", ".join(subs) if subs else "None"

        print(f"{t:<60} | {msg_type:<40} | {pub_str:<40} | {sub_str:<40}")

if __name__ == '__main__':
    inspect_topics()
