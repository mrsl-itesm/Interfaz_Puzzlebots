from django.shortcuts import render, redirect
from .forms import PuzzlebotForm

def agregar_puzzlebot(request):
    if request.method == "POST":
        form = PuzzlebotForm(request.POST)
        print("FOrm recibido")
        if form.is_valid():
            form.save()
            return redirect('monitor')
    else:
        form = PuzzlebotForm()
    return render(request, 'configuracion/configurar.html', {'form': form})

