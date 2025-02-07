from django.urls import path
from . import views

urlpatterns = [
    path('', views.agregar_puzzlebot, name='agregar_puzzlebot'),
]