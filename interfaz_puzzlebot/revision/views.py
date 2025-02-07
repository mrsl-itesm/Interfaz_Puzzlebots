from django.shortcuts import render
from monitoreo.models import Puzzlebot

def check_puzzlebot(request):
    puzzlebots = Puzzlebot.objects.all()
    return render(request, 'revision/revisar.html', {'puzzlebots': puzzlebots})
