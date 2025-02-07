import subprocess
import platform

class Puzzlebot:
    def __init__(self, name, ip):
        self.name = name
        self.ip = ip
        self.vicon = "Apagado"
        self.status = "Desconectado"
    
    def ping(self):
        host = self.ip
        # Determinar el comando según el sistema operativo
        param = "-n" if platform.system().lower() == "windows" else "-c"
        comando = ["ping", param, "1", host]
        try:
            subprocess.run(comando, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
            return True
        except subprocess.CalledProcessError:
            return False
