# configuracion.py
#!/usr/bin/env python3

"""

"""

import rclpy # ROS2 client library
from rclpy.node import Node # ROS2 node class
import asyncio # Async
import websockets # WebSocket client
import json # JSON for data
import requests
import os # OS for system calls
import urllib3 # To disable HTTPS verification warnings

class ConfigurationWebSocketBridge(Node):
    def __init__(self, ws_uri):
        super().__init__('configuracion_websocket')
        self.ws_uri = ws_uri
        self.websocket_task_started = False
        # Starts the WebSocket handler after 1 second only once via the flag above
        self.create_timer(1.0, self.start_websocket_handler)
    
    def start_websocket_handler(self):
        if not self.websocket_task_started: # If the websocket task has not been started
            self.websocket_task_started = True # Change the flag's value to True
            asyncio.create_task(self.websocket_handler()) # Create an asynchronpus task to handle WebSocket connection

    async def websocket_handler(self):
        while rclpy.ok(): # If ROS2 is active
            try:
                async with websockets.connect(self.ws_uri) as ws: # Connect asynchronously to a WebSocket server
                    self.get_logger().info("Conectado al WebSocket.")
                    while rclpy.ok(): # While ROS2 is still active
                        try:
                            response = await asyncio.wait_for(ws.recv(), timeout=10.0) # Gets message from the WebSocket server while waiting 10 seconds
                            data = json.loads(response) # Message is received and stored, expected to be in JSON format
                            # An example of the expected data is: {"nombre": "pzbt1", "ip": "10.15.232.11"}
                            self.get_logger().info(f"Datos recibidos del WebSocket: {data}.")

                            name = data["nombre"]
                            ip = data["ip"]
                            try:
                                with open("nodos/puzzlebots.json", "r+") as file: # Open the local JSON file with puzzlebot data
                                    puzzlebots = json.load(file) # Load the data of the puzzlebots.json file
                                    puzzlebots[name] = ip # Adds the entry for the given Puzzlebot name with the new IP from the WebSocket server
                                    file.seek(0) # File is rewritten
                                    json.dump(puzzlebots, file, indent=4)
                                    file.truncate()
                                    self.get_logger().info(f"Puzzlebot {name} configurado.")
                            except Exception as e:
                                self.get_logger().error(f"Error {e}.")

                            try:
                                self.configure(name, ip) # Calls method to configure topic names, Jetson hotspot IP, and Wi-Fi connection
                            except Exception as e:
                                self.get_logger().error(f"Error {e}.")
                        except:
                            pass
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                self.get_logger().error(f"WebSocket desconectado: {e}, intenando reconectar...")
                await asyncio.sleep(5)

    def configure(self, nombre, ip):
        ruta_sd = "/media/main/063c7ad1-b8d1-4874-a7d5-c348d08fc0ea"
        # https://192.168.1.1/config
        self.agregar_wifi(ruta_sd + "/etc/network/interfaces", ip) # Sets static IP Wi-Fi config
        config_json = self.generate_config_json(nombre) # Generate topic remapping config JSON with robot-specific prefix
        self.upload_config_to_puzzlebot(ip, config_json) # Upload topic config JSON to the robot via its advanced config page

    # TO DO: 

    def generate_config_json(self, prefix):
        # Config structure with robot-specific topic prefixes
        new_config = {
            "prefix": "robot1",
            "topics": {
                "cmd_vel": "/robot1/cmd_vel",
                "VelEncR": "/robot1/VelEncR",
                "VelEncL": "/robot1/VelEncL"
            },
            "other_setting": "value"
        }
        self.get_logger().info(f"Generated config for {prefix}: {new_config}")
        return new_config

    def upload_config_to_puzzlebot(self, ip, config_json): # Modifies topic names via Hackerboard advanced configuration page
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
        # Assuming the web server exposes a REST API endpoint and accepts POST request with JSON payload
        # Make sure the robot's API is enabled for POST requests with JSON
        url = "https://ip/api/upload_config" # Replace the path if needed. Need dynamic URL
        try:
            response = requests.post(url, json=config_json, verify=False)
            if response.status_code == 200:
                self.get_logger().info(f"Configuration uploaded to {ip} successfully.")
            else:
                self.get_logger().error(f"Failed to upload config to {ip}. Status: {response.status_code} Response: {response.text}")
        except requests.RequestException as e:
            self.get_logger().error(f"Error uploading config to {ip}: {e}")

    def agregar_wifi(self, ruta_interfaces, ip):
        ssid = "Lab_Robotica"
        password = "coke.fanta.sprite"

        nueva_red = f'''auto wlan0
            iface wlan0 inet static
                address {ip}
                netmask 255.255.255.0
                gateway 10.15.232.127
                wpa-ssid "{ssid}"
                wpa-psk "{password}"
            '''

        sudo_password = "mrsl-plz123!#"
        os.system(f'echo "{sudo_password}" | sudo -S rm -f {ruta_interfaces}')
        with open("/tmp/interfaces", "w") as file:
            file.write(nueva_red)
        os.system(f'echo "{sudo_password}" | sudo -S mv /tmp/interfaces {ruta_interfaces}')
        os.system(f'echo "{sudo_password}" | sudo -S chmod 644 {ruta_interfaces}')
        print("Red WiFi agregada con IP estática usando sudo.")

def main(args=None):
    rclpy.init(args=args) # Starts ROS2
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/configuracion" # Interface configuration page
    configuracion = ConfigurationWebSocketBridge(ws_uri) # Bridge to WebSocket server instance

    try:
        rclpy.spin(configuracion) # Run the node's event loop (start the websocket handler)
    except KeyboardInterrupt:
        pass
    finally:
        configuracion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()