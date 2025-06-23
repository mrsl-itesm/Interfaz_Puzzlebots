# monitoreo.py
#!/usr/bin/env python3

""" 
This node acts as a real-time ROS2-based monitoring system for the Puzzlebots allowing the controller
configuracion.py to track operational status information about Vicon and ROS2 connection and send it
to the WebSocket server for visualization
"""

import rclpy # ROS2 Python client library
from rclpy.node import Node # Base class for ROS2 nodes
from rclpy.duration import Duration # Handles timestamps
from rclpy.time import Time as RclpyTime # Handles timestamps
from builtin_interfaces.msg import Time # Handles timestamps
import asyncio # Async communication
import websockets # Async communication
import json # Encode / decode JSON data
import time # Blocking sleeps
from geometry_msgs.msg import TransformStamped # ROS2 message type for Pose description (from Vicon)
from std_msgs.msg import Float32 # ROS2 message type for float values
from puzzlebot import Puzzlebot # Custom Puzzlebot class

# ROS2 node that monitors Puzzlebots and sends data over a WebSocket.
class MonitoreoWebSocketBridge(Node):
    # Constructor takes WebSocket URI, list of Puzzlebot names and IPs.
    def __init__(self, ws_uri, names, ips):
        super().__init__('monitoreo_websocket') # Initializes the parent Node with the name 'monitoreo_websocket'
        self.ws_uri = ws_uri # Stores the WebSocket URI
        self.puzzlebots = [Puzzlebot(name, ip) for name, ip in zip(names, ips)] # List of Puzzlebot instances pairing each name with its IP
        
        # Iterates over a list of puzzlebot instances
        for robot in self.puzzlebots:
            # Subscribes to Vicon topic to track Puzzlebot poses
            self.create_subscription(
                TransformStamped,
                f"/vicon/{robot.name}/{robot.name}",
                self.create_vicon_callback(robot),
                10
            )
            # Subscribes to left wheel velocity topic for each Puzzlebot
            self.create_subscription(
                Float32,
                f"{robot.name}/wl",
                self.create_ros_callback(robot),
                10
            )
            # Timestamps as initial values for message tracking
            robot.last_vicon_message = self.get_clock().now() # Records the current time as the last received message time
            robot.last_ros_message = self.get_clock().now()

        # Sets flag to prevent duplicate WebSocket tasks
        self.websocket_task_started = False
        # Timer that runs every second to check if data is being received and the robot status through handler
        self.create_timer(1.0, self.start_websocket_handler)
    
    # Logs a message and returns a lambda that updates the last_ros_message timestamp for the Puzzlebot when a message is received
    def create_ros_callback(self, robot):
        return lambda msg : (
            setattr(robot, "last_ros_message", self.get_clock().now()),
            self.get_logger().info("Mensage de ROS recibido.")
        )
    
    # Similar lambda for updating the Vicon message timestamp
    def create_vicon_callback(self, robot):
        return lambda msg: setattr(robot, "last_vicon_message", self.get_clock().now())
    
    # Status-checking method
    def check_status(self):
        now = self.get_clock().now() # Initializes status-checking timestamp
        # Iterates through all puzzlebot instances in the list
        for robot in self.puzzlebots:
            # Verifies Vicon state by calculating the difference between a new time stamp and the one from the last Vicon message
            diff_vicon = (now.nanoseconds - robot.last_vicon_message.nanoseconds) / 1e9
            robot.vicon = "Encendido" if diff_vicon <= 2 else "Apagado"

            # Similar to the previous line of code but instead verifies the ROS connection state
            diff_ros = (now.nanoseconds - robot.last_ros_message.nanoseconds) / 1e9
            # If there is a valid result from the ping performed by the current Puzzlebot instance
            if robot.fast_ping():
                robot.status = "Conectado" if diff_ros <= 5 else "No ROS" # If the difference is less than or equa to 5 seconds, ROS2 is connected
            # Else, ROS2 is disconnected
            else:
                robot.status = "Desconectado"
            time.sleep(0.1) # Spacing between ping checks

    # Function that initializes the WebSocket server connection
    def start_websocket_handler(self):
        # If the WebSocket task hasn't been started
        if not self.websocket_task_started:
            self.websocket_task_started = True # The flag is set to True
            asyncio.create_task(self.websocket_handler()) # Starts only one WebSocket handler and runs it as an async task        

    # Async function that handles the communication to the WebSocket server from the client
    async def websocket_handler(self):
        # While the ROS2 connection is active
        while rclpy.ok():
            # Connects to the WebSocket server with pings that maintain an open connection between the server and client (this node)
            try:
                async with websockets.connect(self.ws_uri, ping_interval=5, ping_timeout=2) as ws:
                    self.get_logger().info("Conectado al WebSocket.") # Logs message that WebSocket connection was successful
                    # While the ROS2 connection is still active
                    while rclpy.ok():
                        self.check_status() # Runs the status-checking method to check Vicon and ROS status of Puzzlebots
                        # Sets data to send with values from each Puzzlebot
                        data = [
                            {"nombre": r.name, "vicon": r.vicon, "estado": r.status}
                            for r in self.puzzlebots
                        ]
                        await ws.send(json.dumps(data)) # Sends the data as a JSON message over WebSocket
                        self.get_logger().info(f"Datos enviados: {data}") # Logs the sent data
                        await asyncio.sleep(2) # Waits 2 seconds before repeating
            # Handles dropped WebSocket connection: Logs it, waits 1 second, retries.
            except (websockets.exceptions.ConnectionClosed, OSError, asyncio.TimeoutError) as e:
                self.get_logger().warn(f"WebSocket desconectado: {e}, intenando reconectar...") # Communicates disconnected WebSocket
                await asyncio.sleep(1) # 

# Main function that sets up and spins the ROS2 node
def main(args=None):
    rclpy.init(args=args) # Initializes the ROS2 client library
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/monitoreo"  # Sets the WebSocket URI
    # Loads Puzzlebot info from a JSON file
    with open("nodos/puzzlebots.json", "r") as file:
        puzzlebots = json.load(file)
    # Creates the node with the names and IPs
    configuracion = MonitoreoWebSocketBridge(ws_uri, list(puzzlebots.keys()), list(puzzlebots.values()))
    # Spins the node , keeps it alive and responsive
    try:
        rclpy.spin(configuracion)
    # Graceful shutdown
    except KeyboardInterrupt:
        pass
    # Shuts down the node and the ROS2 system
    finally:
        configuracion.destroy_node()
        rclpy.shutdown()

# Ensures main() runs when script is executed directly
if __name__ == '__main__':
    main()