# Interfaz_Puzzlebots
Interfaz para configurar los Puzzlebots

# Pasos para configurar un nuevo Puzzlebot

1. Instalar la imagen del Puzzlebot en la Jetson(Revisar guia para descargar la imagen)
2. Conectar una antena en un puerto USB del Puzzlebot
3. Abrir una terminal y correr los siguientes comandos
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
