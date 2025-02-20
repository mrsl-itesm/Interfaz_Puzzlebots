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
    
    def check_and_update_remote_file(self, host, username, password, file_path, ips):
        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        try:
            ssh_client.connect(host, username=username, password=password)
            for search_key, expected_value in ips.items():
                # Leer el archivo remoto
                command = f'grep "^export {search_key}=" {file_path}'
                stdin, stdout, stderr = ssh_client.exec_command(command)
                output = stdout.read().decode().strip()
                
                if not output:
                    print(f"La clave 'export {search_key}' no fue encontrada en {file_path}.")
                    continue
                
                # Extraer el valor después del '='
                line_parts = output.split('=')
                if len(line_parts) < 2:
                    print(f"Formato incorrecto en la línea encontrada: {output}")
                    continue
                
                actual_value = line_parts[1].strip()
                
                # Comparar valores y actualizar si es necesario
                if actual_value != expected_value:
                    update_command = f"sed -i 's|export {search_key}={actual_value}|export {search_key}={expected_value}|' {file_path}"
                    ssh_client.exec_command(update_command)
                    print(f"Valor actualizado: export {search_key}={expected_value}")
                else:
                    print(f"El valor ya es el esperado: {actual_value}")
            
            ssh_client.close()
        
        except Exception as e:
            return f"Error: {str(e)}"
    
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
                                rosmaster = data["rosmasterip"]
                                ips = {
                                    "ROS_IP": ip,
                                    "ROS_MASTER_URI": "http://"+rosmaster+":11311"
                                }
                                rospy.loginfo(f"Revisando datos del Puzzlebot")
                                self.check_and_update_remote_file(ip, 'puzzlebot', 'Puzzlebot72', 'catkin_ws/src/puzzlebot_autostart/scripts/puzzlebot_start.sh', ips)                           
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