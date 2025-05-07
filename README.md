# Overview
The system is a ROS2-based, WebSocket-enabled interface that allows a centralized management of Puzzlebots. It performs:
- Configuration of individual robots (Wi-Fi, launch files, environment variables).
- Remote updating or rebooting via SSH.
- Monitoring robot status (connectivity, sensor activity).
- All commands come from a central Web Interface via WebSocket.

# Main Components
configuracion.py
    Handles robot setup (Wi-Fi, IP, environment variables, startup configuration).
revision.py
    Reviews or reboots robots via SSH based on WebSocket commands.
monitoreo.py
    Monitors robot status, publishes to WebSocket(connectivity, sensors)
Web Interface (dashboard)
    Sends commands (via WebSocket), receives Puzzlebot status.
puzzlebots.json
    Local JSON file tracking Puzzlebot names and IPs.

# Workings
1. User connects Puzzlebot to interface
- Interface sends IP/name to configuracion.py.
- Puzzlebot gets IP, Wi-Fi, startup config updated.
2. User asks to "check" or "restart" a  Puzzlebot
- Interface sends command to revision.py.
- Node logs in via SSH, updates config or restarts robot.
3. Interface dashboard auto-refreshes
- monitoreo.py streams Puzzlebot status to WebSocket.
- Dashboard allows live status of all connected Puzzlebots.

# Interfaz_Puzzlebots
Interfaz para configurar los Puzzlebots

# Pasos para configurar un nuevo Puzzlebot

1. Instalar la imagen del Puzzlebot en la Jetson(Revisar guia para descargar la imagen).

2. Conectar una antena en un puerto USB del Puzzlebot.

### TP-Link AC1300 Interface Connection

To install the drivers for the TP-Link T3U Plus AC1300 Wireless Adapter:

- Plug in the device and ensure it is detected via ```lsub```.
For the T3U Plus, the ID is 2357:0138.
Consult instructions from [RTL88x2BU-Linux-Driver](https://github.com/RinCat/RTL88x2BU-Linux-Driver), run as sudo:
```
git clone "https://github.com/RinCat/RTL88x2BU-Linux-Driver.git" /usr/src/rtl88x2bu-git
sed -i 's/PACKAGE_VERSION="@PKGVER@"/PACKAGE_VERSION="git"/g' /usr/src/rtl88x2bu-git/dkms.conf
dkms add -m rtl88x2bu -v git
dkms autoinstall
```

Reboot.

Also consult this [WiFi Adapter TPLink AC1300](https://forums.developer.nvidia.com/t/wifi-adapter-tplink-ac1300/243480) forum discussion: 

Connect the Puzzlebot's Jetson Nano to your external PC via micro-USB to create a network interface on both devices. The IP of the interface that connects via USB to the Nano will likely be 192.168.55.x (e.g., 192.168.55.1). To determine the address, view the Connection Information page displayed in the Jetson Nano.

Then, in your external PC compress the rtl88x2bu directory into a zip file and copy the file to the Jetson Nano like this:


```scp rtl88x2bu.zip username@192.168.55.1:/home/username```

Replace username with your Jetson Nano's username such as 'puzzlebot'. When using the micro USB interface, the Jetson will locally always have the address '192.168.55.1', so use it for this step.

After that, change directory to the folder and launch the following commands. The first one takes time and will display a lot of warnings:

```
make
sudo make install
sudo modprobe 88x2bu
sudo reboot
```

Connect your TP-Link AC1300 device to a USB port and connect to your desired Wi-Fi connection.

3. Abrir una terminal y correr los siguientes comandos.

## Primera Vez usando la interfaz
```
git clone https://github.com/Multi-Robot-Systems-Laboratory-ITESM/Interfaz_Puzzlebots.git
```
## Siempre
```
cd Interfaz_Puzzlebots
python3 mrsl_puzzlebot.py
```
4.  Ingresar los siguientes datos
    1. Nombre que se le quiere dar al Puzzlebot
    2. Direccion IP que se le quiere asignar
    3. Red a la que se quiere conectar
    4. Direccion IP del master(Dispositivo que correra el roscore)
5. Dar click en el boton Configurar Puzzlebot
6. Una vez terminado el Puzzlebot estara listo para usarse

En caso de fallas o dudas contactar a un miembro del laboratorio

## Correr Interfaz desde Computadora Principal de MRSL

Pull the ROS2 Humble Docker image

``` docker pull osrf/ros:humble-desktop ```

Run the Docker container

``` docker run -it --rm --net=host osrf/ros:humble-desktop ```

-it: Runs the container in interactive mode, allowing you to interact with the shell.
--net=host: Uses the host network, which is necessary for ROS2 communication.

