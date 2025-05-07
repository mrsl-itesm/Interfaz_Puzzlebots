# edicion.py
#!/usr/bin/env python3

""" 
Connects to a WebSocket server, listens for instructions (like "edit this Puzzlebot's config), and
prepares to apply them to specific Puzzlebots using SSH all from within a ROS2 node
 The web dashboard from the Multi-Robotic Systems Laboratory sends out config or restart commands via WebSocket
 This code receives those commands and securely connects to each Puzzlebot via SSH.
 It updates IPs, ROS master URIs, or restarts services without touching the robot physically.
 This code represents the WebSocket client that receives real-time control commands from a central server.
"""

import rclpy # ROS2  Python client library
from rclpy.node import Node # Base Node class
import asyncio # Asynchronous tasks
import websockets # WebSocket communication
import json # JSON data parsing
from geometry_msgs.msg import TransformStamped # ROS2 Message type
from std_msgs.msg import String # RSO2 message type
from puzzlebot import Puzzlebot # Custom Puzzlebot class
import paramiko # Hnadles SSH communication with remote systems

# 
class EdicionWebSocketBridge(Node):
    # Constructor takes WebSocket URI, list of Puzzlebot names and IPs.
    def __init__(self, ws_uri, names, ips):
        super().__init__('edicion_websocket') # Initializes the parent Node with the name 'edicion_websocket'
        self.ws_uri = ws_uri # Stores the WebSocket URI
        self.puzzlebots = [Puzzlebot(name, ip) for name, ip in zip(names, ips)] # List of Puzzlebot instances pairing each name with its IP

        # Start the asyncio task for WebSocket after a small delay to allow the node to fully initialize
        self.create_timer(1.0, self.start_websocket_handler, callback_group=None) # Creates a timer that runs after 1 second to launch the WebSocket handler
        self.websocket_task_started = False # Tracks whether the WebSocket listener has already been started
    
    # Prepares the SSH connection.
    def edit_values(self, host, username, password, file_path, value):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) # Sets policy to auto-accept host keys

        # Attempts to connect to the remote host via SSH
        try:
            ssh_client.connect(host, username=username, password=password)
        # Returns the error if SSH connection fails.
        except Exception as e:
            return f"Error: {str(e)}"
    
    # Starts the WebSocket listener asynchronously once, using asyncio.create_task
    def start_websocket_handler(self):
        if not self.websocket_task_started:
            self.websocket_task_started = True
            asyncio.create_task(self.websocket_handler())
    
    # Async function that handles WebSocket communication
    async def websocket_handler(self):
        # Loops continuously while ROS2 is running.
        while rclpy.ok():
            # Attempts to connect to the WebSocket server
            try:
                async with websockets.connect(self.ws_uri) as ws:
                    self.get_logger().info("Conectado al WebSocket") # Logs message once connected
                    while rclpy.ok():
                        # Waits for a WebSocket message, timing out after 10 seconds
                        try:
                            response = await asyncio.wait_for(ws.recv(), timeout=10.0)
                            data = json.loads(response) # Parses the received JSON into a Python dictionary
                            self.get_logger().info(f"Datos recibidos del WebSocket: {data}")
                            ip = data["ip"]
                            nombre = data["nombre"]
                        # Logs the received data and extracts ip and nombre fields.
                        except asyncio.TimeoutError:
                            continue
                        # Logs any other unexpected WebSocket error.
                        except Exception as e:
                            self.get_logger().info(f"Error interno de WebSocket: {e}")
            # Handles dropped WebSocket connection: Logs it, waits 5 seconds, retries.
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                self.get_logger().warn(f"WebSocket desconectado: {e}, intenando reconectar...")
                await asyncio.sleep(5)

# Main function that sets up and spins the ROS2 node
def main(args=None):
    rclpy.init(args=args) # Initializes the ROS2 client library
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/edicion" # Sets the WebSocket URI
    # Loads Puzzlebot info from a JSON file
    with open("nodos/puzzlebots.json", "r") as file:
        puzzlebots = json.load(file)
    # Creates the node with the names and IPs
    configuracion = EdicionWebSocketBridge(ws_uri, list(puzzlebots.keys()), list(puzzlebots.values()))
    # Spins the node , keeps it alive and responsive
    try:
        rclpy.spin(configuracion)
    # Graceful shutdown
    except KeyboardInterrupt:
        pass
    # Shuts down the node and ROS2 system
    finally:
        configuracion.destroy_node()
        rclpy.shutdown()

# Ensures main() runs when script is executed directly
if __name__ == '__main__':
    main()