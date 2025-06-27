from django.db import models


class Home(models.Model):
    id = models.AutoField(primary_key=True) 
    t1 = models.FloatField()
    t2 = models.FloatField()
    t3 = models.FloatField()
    t4 = models.FloatField()
    t5 = models.FloatField()
    t6 = models.FloatField()

    class Meta:
        ordering = ["id"]

    def __str__(self):
        return f"This is home id: {self.id}"

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
    t1 = models.FloatField()
    t2 = models.FloatField()
    t3 = models.FloatField()
    t4 = models.FloatField()
    t5 = models.FloatField()
    t6 = models.FloatField()
    tool = models.IntegerField()
    figure = models.IntegerField()
    work = models.IntegerField()
    motion = models.CharField()
    ee = models.CharField()
    stop = models.BooleanField()
    vel = models.IntegerField()
    acc = models.IntegerField()
    corner = models.IntegerField()

    class Meta:
        ordering = ['point_id']

    def __str__(self):
        return f"Point {self.point_id} of {self.path.name}"
    
class Aruco(models.Model):
    point = models.ForeignKey(Point, on_delete=models.CASCADE, related_name='aruco_tags')

    id_aruco = models.IntegerField()
    task = models.CharField(max_length=255)
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField()
    roll = models.FloatField()
    pitch = models.FloatField()
    yaw = models.FloatField()

    def __str__(self):
        return f"Aruco {self.id_aruco} (Point {self.point.point_id}, Path {self.point.path.path_id})"


    
class Global(models.Model):
    point_id = models.IntegerField()
    name = models.CharField(max_length=255)
    t1 = models.FloatField()
    t2 = models.FloatField()
    t3 = models.FloatField()
    t4 = models.FloatField()
    t5 = models.FloatField()
    t6 = models.FloatField()
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

class Site(models.Model):
    site_id = models.IntegerField(unique=True)
    name = models.CharField(max_length=255)

    class Meta:
        ordering = ["site_id"]

    def __str__(self):
        return self.name
    
class Map(models.Model):
    site = models.ForeignKey(Site, to_field='site_id', on_delete=models.CASCADE)
    map_id = models.IntegerField()
    name = models.CharField(max_length=255)
    new_file = models.BooleanField()
    name_file = models.CharField(max_length=255)

    class Meta:
        ordering = ['map_id']

    def __str__(self):
        return f"Point {self.map_id} of {self.site.name}"


class Accounts(models.Model):
    username = models.CharField(max_length=255)
    password = models.CharField(max_length=255)
    fullname = models.CharField(max_length=255)
    gmail = models.CharField(max_length=255)

    class Meta:
        ordering = ['username']

    def __str__(self):
        return f"Its work with {self.username}"
