# Step 1: Connect to Puzzlebot Hotspot

import subprocess

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

# Step 2: Access 192.169.1.1 via Python

import requests

try:
    response = requests.get("http://192.168.1.1/config")
    print("Page content:")
    print(response.text)
except requests.exceptions.RequestException as e:
    print("Could not reach Puzzlebot:", e)
