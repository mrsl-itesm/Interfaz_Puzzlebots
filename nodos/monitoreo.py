#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import json
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from puzzlebot import Puzzlebot
import subprocess


class MonitoreoWebSocketBridge:
    def __init__(self, ws_uri, names, ips):
        rospy.init_node("monitoreo_websocket")
        self.ws_uri = ws_uri
        self.puzzlebots = [Puzzlebot(name, ip) for name, ip in zip(names, ips)]

        for robot in self.puzzlebots:
            rospy.Subscriber(f"/vicon/{robot.name}/{robot.name}", TransformStamped, self.create_vicon_callback(robot))
            rospy.Subscriber(f"/{robot.name}/wl", String, self.create_ros_callback(robot))
            robot.last_vicon_message = rospy.Time.now()
            robot.last_ros_message = rospy.Time.now()

        rospy.Timer(rospy.Duration(2), self.check_status)
    
    def create_ros_callback(self, robot):
        return lambda msg: setattr(robot, "last_ros_message", rospy.Time.now())

    def create_vicon_callback(self, robot):
        return lambda msg: setattr(robot, "last_vicon_message", rospy.Time.now())
    
    def check_vicon(self, robot):
        if (rospy.Time.now() - self.last_vicon_message).to_sec() > 5:
            robot.vicon = "Apagado"
    
    def check_status(self, event):
        now = rospy.Time.now()
        for robot in self.puzzlebots:
            # Verificar estado de Vicon
            robot.vicon = "Encendido" if (now - robot.last_vicon_message).to_sec() <= 5 else "Apagado"

            # Verificar estado de conexión y ROS
            if robot.ping():
                robot.status = "Conectado" if (now - robot.last_ros_message).to_sec() <= 15 else "No ROS"
            else:
                robot.status = "Desconectado"


    async def websocket_handler(self):
         while not rospy.is_shutdown():
            try:
                async with websockets.connect(self.ws_uri) as ws:
                    rospy.loginfo("Conectado al WebSocket")
                    while not rospy.is_shutdown():
                        data = [
                            {"nombre": r.name, "vicon": r.vicon, "estado": r.status}
                            for r in self.puzzlebots
                        ]
                        await ws.send(json.dumps(data))
                        rospy.loginfo(f"Datos enviados: {data}")
                        await asyncio.sleep(2)
            except (websockets.exceptions.ConnectionClosed, OSError) as e:
                rospy.logwarn(f"WebSocket desconectado: {e}, intentando reconectar...")
                await asyncio.sleep(5)

def main():
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/monitoreo"
    with open("puzzlebots.json", "r") as file:
        puzzlebots = json.load(file)

    monitoreo = MonitoreoWebSocketBridge(ws_uri, list(puzzlebots.keys()), list(puzzlebots.values()))
    asyncio.run(monitoreo.websocket_handler())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass