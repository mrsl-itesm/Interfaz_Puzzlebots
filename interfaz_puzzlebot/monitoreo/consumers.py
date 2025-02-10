from channels.generic.websocket import JsonWebsocketConsumer
from monitoreo.models import Puzzlebot
from asgiref.sync import async_to_sync

class MonitoreoConsumer(JsonWebsocketConsumer):
    def connect(self):
        # Asignar al cliente a un grupo
        self.path = self.scope["path"]
        self.group_name = "monitoreo_group"
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
        if self.path == "/ws/puzzlebots/monitoreo":
            self.handle_monitoreo(content)
    
    def handle_monitoreo(self, content):
        for element in content:
            puzzlebot_id = element.get('nombre')
            vicon = element.get('vicon')
            estado = element.get('estado')
            try:
                # Busca el puzzlebot y actualiza el campo 'cantidad'
                puzzlebot = Puzzlebot.objects.get(nombre=puzzlebot_id)
                puzzlebot.vicon = vicon
                puzzlebot.estatus = estado
                puzzlebot.save()
                async_to_sync(self.channel_layer.group_send)(
                    self.group_name,
                    {
                        "type": "monitoreo_message",  # Tipo de mensaje manejado por el grupo
                        "nombre": puzzlebot.nombre,
                        "vicon": puzzlebot.vicon,
                        "estado": puzzlebot.estatus,
                        "status": "success",
                        "message": f"Estado de {puzzlebot.nombre} actualizado a {vicon}",
                    }
                )
            except Puzzlebot.DoesNotExist:
                async_to_sync(self.channel_layer.group_send)(
                    self.group_name,
                    {
                        "type": "monitoreo_message",  # Tipo de mensaje manejado por el grupo
                        "status": "error",
                        "message": "Puzzlebot no encontrado",
                    }
                )
    
    def monitoreo_message(self, event):
        self.send_json({
            "status": event.get("status"),
            "nombre": event.get("nombre"),
            "vicon": event.get("vicon"),
            "estado": event.get("estado"),
            "message": event.get("message"),
        })