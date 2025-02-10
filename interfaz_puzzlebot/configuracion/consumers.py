from channels.generic.websocket import JsonWebsocketConsumer
from configuracion.forms import PuzzlebotForm
from asgiref.sync import async_to_sync

class ConfiguracionConsumer(JsonWebsocketConsumer):
    def connect(self):
        # Asignar al cliente a un grupo
        self.path = self.scope["path"]
        self.group_name = "configuracion_group"
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
        if self.path == "/ws/puzzlebots/configuracion":
            self.handle_configuracion(content)

    def handle_configuracion(self, content):
        name = content.get('nombre')
        ip = content.get('ip')
        async_to_sync(self.channel_layer.group_send)(
            self.group_name,
            {
                "type": "broadcast_message",
                "accion": "agregar",
                "nombre": name,
                "ip": ip,  
            }
        )
        if name and ip:
            form = PuzzlebotForm(content)
            if form.is_valid():
                form.save()
    
    def broadcast_message(self, event):
        # Enviar el mensaje a los clientes
        self.send_json({
            "accion": event.get("accion"),
            "nombre": event.get("nombre"),
            "ip": event.get("ip"),
        })