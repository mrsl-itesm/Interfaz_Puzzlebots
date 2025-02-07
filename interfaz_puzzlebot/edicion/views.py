from django.shortcuts import render
from monitoreo.models import Puzzlebot

def edit_puzzlebots(request):
    puzzlebots = Puzzlebot.objects.all()
    return render(request, 'edicion/editar.html', {'puzzlebots': puzzlebots})
