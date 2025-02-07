from django.shortcuts import render

def monitor(request):
    datos = [
        {"nombre": "pzbt1", "edad": 25, "ciudad": "Madrid"},
        {"nombre": "pzbt2", "edad": 30, "ciudad": "Barcelona"},
        {"nombre": "pzbt3", "edad": 28, "ciudad": "Sevilla"},
    ]
    return render(request, 'inicio/monitor.html', {"datos": datos})

def pagina_principal(request):
    return render(request, 'inicio/pagina_principal.html')

