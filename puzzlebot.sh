#!/bin/bash

python3 interfaz_puzzlebot/manage.py runserver &
python3 nodos/monitoreo.py &
python3 nodos/configuracion.py &
python3 nodos/edicion.py &
python3 nodos/revision.py &

wait