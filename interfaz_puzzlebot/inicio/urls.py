from django.urls import path
from . import views
from . import consumers

urlpatterns = [
    path('', views.monitor, name='monitor'),
]
