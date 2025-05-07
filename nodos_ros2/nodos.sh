#!/bin/bash

# Ejecutar los nodos en paralelo
python3 monitoreo.py &
python3 configuracion.py &
python3 edicion.py &
python3 revision.py &

# Esperar a que los procesos terminen
wait