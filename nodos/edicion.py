#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import json
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from puzzlebot import Puzzlebot
import subprocess

class EdicionWebSocketBridge:
    def __init__(self, ws_uri, names, ips):
        rospy.init_node("monitoreo_websocket")
        self.ws_uri = ws_uri
        self.puzzlebots = [Puzzlebot(name, ip) for name, ip in zip(names, ips)]
    
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

                        except:
                            pass
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                rospy.logwarn(f"WebSocket desconectado: {e}, intentando reconectar...")
                await asyncio.sleep(5)

def main():
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/edicion"
    with open("puzzlebots.json", "r") as file:
        puzzlebots = json.load(file)

    configuracion = EdicionWebSocketBridge(ws_uri, list(puzzlebots.keys()), list(puzzlebots.values()))
    asyncio.run(configuracion.websocket_handler())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass