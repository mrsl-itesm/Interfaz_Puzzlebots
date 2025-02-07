#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import json

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
                                with open("puzzlebots.json", "r+") as file:
                                    puzzlebots = json.load(file)
                                    puzzlebots[name] = ip
                                    file.seek(0)
                                    json.dump(puzzlebots, file, indent=4)
                                    file.truncate()
                                    rospy.loginfo(f"Puzzlebot {name} configurado")
                            except Exception as e:
                                rospy.logerr(f"Error {e}")
                        except:
                            pass
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                rospy.logwarn(f"WebSocket desconectado: {e}, intentando reconectar...")
                await asyncio.sleep(5)

def main():
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/configuracion"
    configuracion = ConfiguracionWebSocketBridge(ws_uri)
    asyncio.run(configuracion.websocket_handler())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass