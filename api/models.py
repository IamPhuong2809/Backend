from django.db import models

class Path(models.Model):
    path_id = models.IntegerField(unique=True)
    name = models.CharField(max_length=255)

    class Meta:
        ordering = ["path_id"]

    def __str__(self):
        return self.name

class Point(models.Model):
    path = models.ForeignKey(Path, to_field='path_id', on_delete=models.CASCADE)
    point_id = models.IntegerField()
    name = models.CharField(max_length=255)
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField()
    roll = models.FloatField()
    pitch = models.FloatField()
    yaw = models.FloatField()
    tool = models.IntegerField()
    figure = models.IntegerField()
    work = models.IntegerField()
    motion = models.CharField()
    cont = models.BooleanField()
    stop = models.BooleanField()
    vel = models.IntegerField()
    acc = models.IntegerField()
    corner = models.IntegerField()

    class Meta:
        ordering = ['point_id']

    def __str__(self):
        return f"Point {self.point_id} of {self.path.name}"
    
class Global(models.Model):
    point_id = models.IntegerField()
    name = models.CharField(max_length=255)
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField()
    roll = models.FloatField()
    pitch = models.FloatField()
    yaw = models.FloatField()
    tool = models.IntegerField()
    figure = models.IntegerField()
    work = models.IntegerField()

    class Meta:
        ordering = ['point_id']

    def __str__(self):
        return f"Point {self.point_id}"

class LoadName(models.Model):
    name = models.CharField(max_length=255, unique=True)

    
class LoadData(models.Model):
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField()
    mass = models.FloatField()
    jx = models.FloatField()
    jxy = models.FloatField()
    jxz = models.FloatField()
    jyx = models.FloatField()
    jy = models.FloatField()
    jyz = models.FloatField()
    jzx = models.FloatField()
    jzy = models.FloatField()
    jz = models.FloatField()


