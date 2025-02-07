from django.db import models

class Puzzlebot(models.Model):
    nombre = models.CharField(max_length=200, unique=True)
    ip = models.CharField(max_length=200)
    vicon = models.CharField(max_length=200)
    estatus = models.CharField(max_length=200)

    def __str__(self):
        return self.nombre
