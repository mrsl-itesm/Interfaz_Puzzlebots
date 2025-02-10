from django.urls import path
from interfaz_puzzlebot.consumers import PuzzlebotConsumer
from monitoreo.consumers import MonitoreoConsumer
from configuracion.consumers import ConfiguracionConsumer
from revision.consumers import RevisionConsumer

websocket_urlpatterns = [
    path('ws/puzzlebots/monitoreo', MonitoreoConsumer.as_asgi()),
    path('ws/puzzlebots/configuracion', ConfiguracionConsumer.as_asgi()),
    path('ws/puzzlebots/revision', RevisionConsumer.as_asgi()),
]
