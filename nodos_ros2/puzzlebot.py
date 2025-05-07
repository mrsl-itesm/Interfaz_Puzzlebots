import subprocess # Library for running shell commands
import platform # Library for OS info
import socket # Used to open a low-level network connection
import os # For interacting with the operating system

# Represents Puzzlebot robot with:
class Puzzlebot:
    def __init__(self, name, ip):
        self.name = name # Name
        self.ip = ip # IP address
        self.vicon = "Apagado" # Motion capture system status
        self.status = "Desconectado" # Connectivity status

    # Sets a connection to check Puzzlebot IP on port 22 SSH
    def fast_ping(self, timeout=0.2):
        host = self.ip # IP address of Puzzlebot to check on port 22 SSH
        port = 22 # Can change it depending on the robot (Ex: 80 for HTTP) SSH default

        # Tries to open a TCP connection to the robot's IP and port
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        # If connection fails or times out, returns False
        except (socket.timeout, OSError):
            return False