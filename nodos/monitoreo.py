#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import json
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from puzzlebot import Puzzlebot


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

        rospy.Timer(rospy.Duration(1), self.check_status)
    
    def create_ros_callback(self, robot):
        return lambda msg: setattr(robot, "last_ros_message", rospy.Time.now())

    def create_vicon_callback(self, robot):
        return lambda msg: setattr(robot, "last_vicon_message", rospy.Time.now())
    
    def check_status(self, event):
        now = rospy.Time.now()
        for robot in self.puzzlebots:
            # Verificar estado de Vicon
            robot.vicon = "Encendido" if (now - robot.last_vicon_message).to_sec() <= 2 else "Apagado"

            # Verificar estado de conexión y ROS
            if robot.fast_ping():
                robot.status = "Conectado" if (now - robot.last_ros_message).to_sec() <= 2 else "No ROS"
            else:
                robot.status = "Desconectado"
            rospy.sleep(0.1)

    async def websocket_handler(self):
         while not rospy.is_shutdown():
            try:
                async with websockets.connect(self.ws_uri, ping_interval=5, ping_timeout=2) as ws:
                    rospy.loginfo("Conectado al WebSocket")
                    while not rospy.is_shutdown():
                        self.check_status(None)
                        data = [
                            {"nombre": r.name, "vicon": r.vicon, "estado": r.status}
                            for r in self.puzzlebots
                        ]
                        await ws.send(json.dumps(data))
                        rospy.loginfo(f"Datos enviados: {data}")
                        await asyncio.sleep(2)
            except (websockets.exceptions.ConnectionClosed, OSError, asyncio.TimeoutError) as e:
                rospy.logwarn(f"WebSocket desconectado: {e}, intentando reconectar...")
                await asyncio.sleep(1)

def main():
    ws_uri = "ws://127.0.0.1:8000/ws/puzzlebots/monitoreo"
    with open("puzzlebots.json", "r") as file:
        puzzlebots = json.load(file)

    monitoreo = MonitoreoWebSocketBridge(ws_uri, list(puzzlebots.keys()), list(puzzlebots.values()))
    loop = asyncio.get_event_loop()
    loop.run_until_complete(monitoreo.websocket_handler())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass