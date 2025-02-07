from django import forms
from monitoreo.models import Puzzlebot

class PuzzlebotForm(forms.ModelForm):
    class Meta:
        model = Puzzlebot
        fields = ['nombre', 'ip']