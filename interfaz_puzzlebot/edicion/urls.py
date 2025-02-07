from django.urls import path
from . import views

urlpatterns = [
    path('', views.edit_puzzlebots, name='editar_puzzlebots'),
]