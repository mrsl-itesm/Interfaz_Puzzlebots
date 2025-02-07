from django.urls import path
from interfaz_puzzlebot.consumers import PuzzlebotConsumer

websocket_urlpatterns = [
    path('ws/puzzlebots/monitoreo', PuzzlebotConsumer.as_asgi()),
    path('ws/puzzlebots/configuracion', PuzzlebotConsumer.as_asgi()),
    path('ws/puzzlebots/edicion', PuzzlebotConsumer.as_asgi()),
    path('ws/puzzlebots/revision', PuzzlebotConsumer.as_asgi()),
]
