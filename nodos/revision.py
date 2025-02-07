#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import json
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from puzzlebot import Puzzlebot
import paramiko

class RevisionWebSocketBridge:
    def __init__(self, ws_uri):
        rospy.init_node("revision_websocket")
        self.ws_uri = ws_uri

    def ejecutar_comando_ssh(self, host, username, password, comando):
        # Crea una instancia de cliente SSH
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            # Conecta al host remoto
            ssh_client.connect(hostname=host, username=username, password=password)
            rospy.loginfo("Conectado")

            #sftp = ssh_client.open_sftp()
            #sftp.put(<Source>, destination_i_CAN_write_to)
            #sftp.close()

            #now move the file to the sudo required area!
            stdin, stdout, stderr = ssh_client.exec_command(comando)
            stdin.write("Puzzlebot72\n")
            stdin.flush()
            rospy.loginfo("Comando ejecutado")

        except Exception as e:
            rospy.logerr(f"Error al conectar o ejecutar el comando: {e}")
        finally:
            # Cierra la conexión SSH
            ssh_client.close()
 
    
    async def websocket_handler(self):
         while not rospy.is_shutdown():
            try:
                async with websockets.connect(self.ws_uri) as ws:
                    rospy.loginfo("Conectado al WebSocket")
                    while not rospy.is_shutdown():
                        try:
                            response = await asyncio.wait_for(ws.recv(), timeout=10.0)
                            data = json.loads(response)
                            rospy.loginfo(f"Datos recibidos del WebSocket: {data}")
                            ip = data["ip"]
                            nombre = data["nombre"]
                            print(ip)
                            if data["accion"] == "revisar":
                                pass
                            elif data["accion"] == "reiniciar":
                                rospy.loginfo(f"Reiniciando {nombre}")
                                comando = 'sudo -S systemctl restart puzzlebot.service'
                                self.ejecutar_comando_ssh(ip, 'puzzlebot', 'Puzzlebot72', comando)
                                rospy.loginfo(f"{nombre} reiniciado")

                        except:
                            pass
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                rospy.logwarn(f"WebSocket desconectado: {e}, intentando reconectar...")
                await asyncio.sleep(5)

def main():
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/revision"
    configuracion = RevisionWebSocketBridge(ws_uri)
    asyncio.run(configuracion.websocket_handler())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass