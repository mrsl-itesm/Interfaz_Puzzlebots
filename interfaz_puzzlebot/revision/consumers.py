from channels.generic.websocket import JsonWebsocketConsumer
from monitoreo.models import Puzzlebot
from asgiref.sync import async_to_sync

class RevisionConsumer(JsonWebsocketConsumer):
    def connect(self):
        # Asignar al cliente a un grupo
        self.path = self.scope["path"]
        self.group_name = "revision_group"
        async_to_sync(self.channel_layer.group_add)(
            self.group_name,
            self.channel_name
        )
        self.accept()  # Aceptar la conexión WebSocket

    def disconnect(self, close_code):
        async_to_sync(self.channel_layer.group_discard)(
            self.group_name,
            self.channel_name
        )
    
    def receive_json(self, content, **kwargs):
        if self.path == "/ws/puzzlebots/revision":
            self.handle_revision(content)

    def handle_revision(self, content):
        accion = content.get('accion')
        name = content.get('nombre')
        ip = content.get('ip') 
        async_to_sync(self.channel_layer.group_send)(
            self.group_name,
            {
                "type": "broadcast_message",
                "accion": accion,
                "nombre": name,
                "ip": ip,  
            }
        )
    
    def broadcast_message(self, event):
        # Enviar el mensaje a los clientes
        self.send_json({
            "accion": event.get("accion"),
            "nombre": event.get("nombre"),
            "ip": event.get("ip"),
        })