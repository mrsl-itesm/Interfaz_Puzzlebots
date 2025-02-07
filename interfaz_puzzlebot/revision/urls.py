from django.urls import path
from . import views

urlpatterns = [
    path('', views.check_puzzlebot, name='revisar_puzzlebots'),
]