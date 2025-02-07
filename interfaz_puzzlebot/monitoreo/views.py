from django.shortcuts import render
from .models import Puzzlebot

def show_puzzlebots(request):
    puzzlebots = Puzzlebot.objects.all()
    return render(request, 'monitoreo/monitor.html', {'puzzlebots': puzzlebots})
