# revision.py
#!/usr/bin/env python3

""" 
Acts as the central ROS-2 integrated controller that:
    1. Receives robot commands from a WebSocket server.
    2. Processes those commands in real-time: Reviews or restarts robot.
    3. Interfaces with ROS2 (indirectly via a ROS2 node and rclpy).
    4. Acts as a link between the network and ROS system:
        - WebSocket: Receives instructions
        - SSH: Sends commands to Puzzlebots
        - ROS2 Node: Manages lifecycle, logging, and potential integration
    The controller enables a centralized, remote control of Puzzlebots
    Handles configuration and rebooting without direct physical access

    configuracion.py is the brain of the system:
        - Listens for external commands via WebSocket.
        - Performs actions on Puzzlebots via SSH.
        - Uses ROS2 for lifecycle and logging.
"""

import rclpy # ROS2 Python client library
from rclpy.node import Node # Import ROS2 node class to create nodes.
import asyncio # Asynchronous programming library for WebSocket communication
import websockets # Used for async, bi-directional communication between client / server
import json # Used to parse or generate JSON data, common for sending structured data over WebSocket
from geometry_msgs.msg import TransformStamped # Message type used to represent transformations in space (position + orientation with a timestamp)
from std_msgs.msg import String # Standard string message type to publish or subscribe text messages
from puzzlebot import Puzzlebot # Imports custom Puzzlebot class
import paramiko # Library for SSH communication, to securely connect and run commands on remote machines

# Class that serves as a bridge between a ROS2 node and a WebSocket server.
# Enables real-time control or monitoring of the Puzzlebots.
class RevisionWebSocketBridge(Node):  # <-- now inherits from Node directly
    # Creates the bridge object and stores Web URI (Uniform Resource Identifier) for connection.
    def __init__(self, ws_uri):
        super().__init__('revision_websocket')
        self.ws_uri = ws_uri
        self.websocket_task_started = False
        self.create_timer(1.0, self.start_websocket_handler)  # small delay before launching the handler

    # Starts the WebSocket listener as a background asyncio task
    def start_websocket_handler(self):
        if not self.websocket_task_started:
            self.websocket_task_started = True
            asyncio.create_task(self.websocket_handler())

    # Method that executes a command over SSH on remote machine.
    def ejecutar_comando_ssh(self, host, username, password, comando):
        # Creates a new SSH client object
        ssh_client = paramiko.SSHClient()
        # Adds SSH host keys (avoiding manual confirmation)
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            # Connects to the specified remote machine using SSH
            ssh_client.connect(hostname=host, username=username, password=password)
            self.get_logger().info("Conectado.")

            stdin, stdout, stderr = ssh_client.exec_command(comando)
            # Execute command on remote machine
            # Writes password into the stdin of the remote command
            stdin.write("Puzzlebot72\n")
            stdin.flush()
            self.get_logger().info("Comando ejecutado.")
            # Reads and decodes the output and errors from the SSH command
            output = stdout.read().decode().strip()
            error = stderr.read().decode().strip()
            # Close the SSH connection and returns output and error
            ssh_client.close()
            return output, error

        # Logs and returns any error that occurs during SSH connection or command execution
        except Exception as e:
            self.get_logger().error(f"Error al conectar o ejecutar el comando: {str(e)}.")
            return None, str(e)

        # Ensures the SSH connection is always closed, even if an error occurs
        finally:
            ssh_client.close()

    # Function that connects to a remote host using SSH
    # file_path is the path to the file to check/update.
    # ips is a dictionary of key-value pairs (KEY=VALUE) that should be set in the file.
    def check_and_update_remote_file(self, host, username, password, file_path, ips):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            ssh_client.connect(host, username=username, password=password)
            # Iterates over each KEY=VALUE pair in the ips dictionary.
            for search_key, expected_value in ips.items():
                # Builds a command to search for a line that starts with "export KEY=" in the file.
                comando = f'grep "^export {search_key}=" {file_path}'
                # Command on the remote machine is executed and the output is read
                stdin, stdout, stderr = ssh_client.exec_command(comando)
                output = stdout.read().decode().strip()

                # If the line is not found, it notifies and skips to the next key.
                if not output:
                    print(f"La clave 'export {search_key}' no fue encontrada en {file_path}.")
                    continue
                    
                # Extract the value after the '='. KEY and VALUE are separated.
                line_parts = output.split('=')

                # If the line has less than 2 parts (NO KEY or VALUE), it is notified that it is in the incorrect format.
                if len(line_parts) < 2:
                    print(f"Formato incorrecto en la línea encontrada: {output}.")
                    continue
                
                # The VALUE from the KEY is extracted from the line.
                actual_value = line_parts[1].strip()

                # Compares the current value in the file with the expected one.
                if actual_value != expected_value:
                    # Builds a 'sed' command to update the line in the file.
                    update_command = f"sed -i 's|export {search_key}={actual_value}|export {search_key}={expected_value}|' {file_path}"
                    # Executes the command and notifies that the value was changed.
                    ssh_client.exec_command(update_command)
                    print(f"Valor actualizado: export {search_key}={expected_value}")
                else:
                    # If the value is already correct, it confirms that.
                    print(f"El valor ya es el esperado: {actual_value}")
        
            # The SSH connection is closed after all updates are processed.
            ssh_client.close()
        
        # Returns a string with the error message if anything fails
        except Exception as e:
            return f"Error: {str(e)}"
    
    # Async function that connects to a WebSocket server at self.ws_uri
    # Waits for JSON messages and depending on the "accion" field:
    # It sets ROS env vars with check_and_update_remote_file() or restarts the robot via SSH
    async def websocket_handler(self):
        # Loop runs as long as the ROS2 system is running
        while rclpy.ok():
            # Tries to connect to the WebSocket server using the URI stored in self.ws_uri
            # async with ensures the connection is properly closed afterwards
            try:
                async with websockets.connect(self.ws_uri) as ws:
                    # Logs a message when the connection is successful
                    self.get_logger().info("Conectado al WebSocket.")
                    # Keeps listening to incoming WebSocket messages until shutdown
                    while rclpy.ok():
                        # Waits for a message from the WebSocket and makes sure it times out after 10 seconds avoiding hangs
                        try:
                            response = await asyncio.wait_for(ws.recv(), timeout=10.0)
                            data = json.loads(response) # Parses the incoming message from JSON format into a Python dictionary
                            self.get_logger().info(f"Datos recibidos del WebSocket: {data}.") # Logs the received data for debugging/inspection
                            ip = data["ip"] # Extracts the IP address from the JSON message
                            nombre = data["nombre"] # Extracts the Puzzlebot name from the JSON message
                            print(ip)
                            # If the action requested is "revisar", it grabs the rosmasterip field
                            if data["accion"] == "revisar":
                                rosmaster = data["rosmasterip"]
                            # Creates a dictionary of environment variables that should be set on the remote Puzzlebot
                                ips = {
                                    "ROS_IP": ip,
                                    "ROS_MASTER_URI": "http://"+rosmaster+":11311"
                                }
                                self.get_logger().info(f"Revisando datos del Puzzlebot.")
                                # Path to the script that contains the environment variables on the remote Puzzlebot
                                # Calls earlier function to SSH into the Puzzlebot, check if the variables are correct, and update them if needed
                                # PENDING: FIND ROS2 HUMBLE UBUNTU 22.04 PUZZLEBOT EQUIVALENT TO: 
                                # 'catkin_ws/src/puzzlebot_autostart/scripts/puzzlebot_start.sh'
                                self.check_and_update_remote_file(ip, 'puzzlebot', 'Puzzlebot72', 'catkin_ws/src/puzzlebot_autostart/scripts/puzzlebot_start.sh', ips)
                            # If the action is "reiniciar", logs that the Puzzlebot is being restarted
                            elif data["accion"] == "reiniciar":
                                self.get_logger().info(f"Reiniciando {nombre}.")
                                # Command to restart the Puzzlebot's service (systemcd-based service restart via sudo)
                                comando = 'sudo -S systemctl restart puzzlebot.service'
                                # Executes the command via SSH on the remote Puzzlebot using the earlier method
                                self.ejecutar_comando_ssh(ip, 'puzzlebot', 'Puzzlebot72', comando)
                                self.get_logger().info(f"{nombre} reiniciado.")
                        # If no WebSocket message is received within 10 seconds, this logs a warning
                        except asyncio.TimeoutError:
                            self.get_logger().warn("Timeout esperando datos del WebSocket.")
            # Catches and logs any error during message handling
            except Exception as e:
                self.get_logger().error(f"Error procesando datos del WebSocket: {str(e)}.")
            # If the connection to the WebSocket server is lost
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                # Logs the error and waits 5 seconds before trying to reconnect
                self.get_logger().error(f"WebSocket desconectado: {e}, intenando reconectar...")
                await asyncio.sleep(5)

def main(args=None):
    rclpy.init(args=args) # Initializes ROS2 client library
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/revision" # WebSocket URI to connect to
    node = RevisionWebSocketBridge(ws_uri) # Creates the custom Node
    try:
        rclpy.spin(node) # Spins ROS2 node, while async task runs in background
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()