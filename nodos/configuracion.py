#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import json
import os

class ConfiguracionWebSocketBridge:
    def __init__(self, ws_uri):
        rospy.init_node("configuracion_websocket")
        self.ws_uri = ws_uri

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

                            name = data["nombre"]
                            ip = data["ip"]
                            try:
                                with open("nodos/puzzlebots.json", "r+") as file:
                                    puzzlebots = json.load(file)
                                    puzzlebots[name] = ip
                                    file.seek(0)
                                    json.dump(puzzlebots, file, indent=4)
                                    file.truncate()
                                    rospy.loginfo(f"Puzzlebot {name} configurado")
                            except Exception as e:
                                rospy.logerr(f"Error {e}")
                            try:
                                self.configure(name, ip)
                            except Exception as e:
                                rospy.logerr(f"Error {e}")
                        except:
                            pass
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                rospy.logwarn(f"WebSocket desconectado: {e}, intentando reconectar...")
                await asyncio.sleep(5)
                
    def configure(self, nombre, ip):
        ruta_sd = "/media/main/063c7ad1-b8d1-4874-a7d5-c348d08fc0ea"
        self.modificar_sh(ruta_sd + "/home/puzzlebot/catkin_ws/src/puzzlebot_autostart/scripts/puzzlebot_start.sh", ip)
        self.modificar_launch(ruta_sd + "/home/puzzlebot/catkin_ws/src/puzzlebot_autostart/launch/puzzlebot_autostart.launch", nombre)
        self.agregar_wifi(ruta_sd + "/etc/network/interfaces", ip)
        
    def modificar_sh(self, ruta_sh, ip):
        print(ruta_sh)
        ros_master = "10.15.232.53"
        ros_ip = ip
        
        with open(ruta_sh, 'r') as file:
            lineas = file.readlines()
        
        with open(ruta_sh, 'w') as file:
            for linea in lineas:
                if "export ROS_MASTER_URI=" in linea:
                    file.write(f"export ROS_MASTER_URI=http://{ros_master}:11311\n")
                elif "export ROS_IP=" in linea:
                    file.write(f"export ROS_IP={ros_ip}\n")
                else:
                    file.write(linea)
        print("Archivo .sh actualizado.")

    def modificar_launch(self, ruta_launch, namespace):
        with open(ruta_launch, 'r') as file:
            contenido = file.read()
        nuevo_contenido = f'<?xml version="1.0"?>\n<launch>\n  <group ns="{namespace}">\n    <node name="rosserial" pkg="rosserial_python" type="serial_node.py"/>\n  </group>\n</launch>'
        with open(ruta_launch, 'w') as file:
            file.write(nuevo_contenido)
        print("Archivo .launch actualizado.")

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

def main():
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/configuracion"
    configuracion = ConfiguracionWebSocketBridge(ws_uri)
    asyncio.run(configuracion.websocket_handler())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass