# Generated by Django 5.1.7 on 2025-07-09 10:01

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('api', '0010_goalpose_goal_id'),
    ]

    operations = [
        migrations.CreateModel(
            name='Action',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('typeData', models.IntegerField(choices=[(0, 'Mobile'), (1, 'Manipulator')])),
                ('idData', models.IntegerField()),
            ],
        ),
    ]
