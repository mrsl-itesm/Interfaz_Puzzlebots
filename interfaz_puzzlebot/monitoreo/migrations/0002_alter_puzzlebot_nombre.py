# Generated by Django 4.2.17 on 2025-01-18 00:17

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('monitoreo', '0001_initial'),
    ]

    operations = [
        migrations.AlterField(
            model_name='puzzlebot',
            name='nombre',
            field=models.CharField(max_length=200, unique=True),
        ),
    ]
