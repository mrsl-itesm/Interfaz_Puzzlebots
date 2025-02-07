from channels.generic.websocket import JsonWebsocketConsumer
from monitoreo.models import Puzzlebot
from asgiref.sync import async_to_sync
from configuracion.forms import PuzzlebotForm

# Diccionario global para rastrear los clientes conectados
connected_clients = {}

class PuzzlebotConsumer(JsonWebsocketConsumer):
    def connect(self):
        # Asignar al cliente a un grupo
        self.path = self.scope["path"]
        self.group_name = "broadcast_group"
        async_to_sync(self.channel_layer.group_add)(
            self.group_name,
            self.channel_name
        )
        connected_clients[self.channel_name] = self.scope["client"]
        self.accept()  # Aceptar la conexión WebSocket

    def disconnect(self, close_code):
        async_to_sync(self.channel_layer.group_discard)(
            self.group_name,
            self.channel_name
        )
        # Eliminar al cliente del diccionario global
        if self.channel_name in connected_clients:
            del connected_clients[self.channel_name]

    def receive_json(self, content, **kwargs):
        # Enviar un mensaje al grupo como prueba
        if self.path == "/ws/puzzlebots/monitoreo":
            self.handle_monitoreo(content)
        elif self.path == "/ws/puzzlebots/configuracion":
            self.handle_configuracion(content)
        elif self.path == "/ws/puzzlebots/edicion":
            pass
        elif self.path == "/ws/puzzlebots/revision":
            pass

    
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
                        "type": "broadcast_message",  # Tipo de mensaje manejado por el grupo
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
                        "type": "broadcast_message",  # Tipo de mensaje manejado por el grupo
                        "status": "error",
                        "message": "Puzzlebot no encontrado",
                    }
                )
    
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
            "status": event.get("status"),
            "accion": event.get("accion"),
            "nombre": event.get("nombre"),
            "vicon": event.get("vicon"),
            "estado": event.get("estado"),
            "message": event.get("message"),
            "ip": event.get("ip"),
        })