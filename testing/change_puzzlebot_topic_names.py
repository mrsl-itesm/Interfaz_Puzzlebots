import requests
import json
import subprocess

def add_puzzlebot_name_to_topics(obj):
    """ Recursively add prefix to all topic keys' values"""
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key in topic_keys:
                obj[key] = prefix + "/" + value
            elif isinstance(value, (dict, list)):
                add_puzzlebot_name_to_topics(value)
    elif isinstance(obj, list):
        for item in obj:
            add_puzzlebot_name_to_topics(item)

# ESP32 IP address
ip_address = "192.168.1.1"

# Puzzlebot name prefix
prefix = "pzbt1"

# Step 1: Connect to Puzzlebot Hotspot

# WiFi credentials
ssid = "Puzzlebot"
password = "Puzzlebot72"

# Connect using nmcli
connect_command = f'nmcli dev wifi connect "{ssid}" password "{password}"'

try:
    result = subprocess.run(connect_command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print("Connection successful:", result.stdout)
except subprocess.CalledProcessError as e:
    print("Failed to connect:", e.stderr)

# Step 2: Access 192.169.1.1 config file via Python

import requests

try:
    response = requests.get(f"http://{ip_address}/config_live.json")
    print("Page content:")
    print(response.text)
except requests.exceptions.RequestException as e:
    print("Could not reach Puzzlebot:", e)

# Step 3: Load JSON content

config_text = response.text
config = json.loads(config_text)

# List of topic keys to modify

topic_keys = [
    "TopicCmdVel", "TopicRobotVel",
    "Topic", "TopicRaw"
]

# Step 4: Apply prefix and convert back to text

add_puzzlebot_name_to_topics(config)
updated_config_text = json.dumps(config, indent=4)

# Step 5: Send updated config

files = {'file': ('config_live.json', updated_config_text)}
post_response = requests.post(f"http://{ip_address}/config", files=files)

if post_response.status_code == 200:
    print("Updated config uploaded successfully.")
else:
    print("Failed to upload updated config.")

# restart_response = requests.post(f"https://{ip_address}/restart")

# if restart_response.status_code == 200:
#     print("Robot restart command sent.")
# else:
#     print(f"Failed to send restart command: {restart_response.status_code}")