import subprocess
import platform
import socket

class Puzzlebot:
    def __init__(self, name, ip):
        self.name = name
        self.ip = ip
        self.vicon = "Apagado"
        self.status = "Desconectado"
    
    def fast_ping(self, timeout=0.2):
        host = self.ip
        port = 22  # Puedes cambiarlo según tu robot (Ej: 80 para HTTP)

        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except (socket.timeout, OSError):
            return False