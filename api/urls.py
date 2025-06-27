from django.urls import path
from .views import (
    power_robot,
    move,
    move_path,
    configuration,
    teach_path,
    position_list,
    components,
    maps,
    login
)

urlpatterns = [
    path('login/', login.Login, name='login'),

    path('EMG/', components.EMG, name='EMG'),
    
    path('O0000/', power_robot.O0000, name='O0000'),
    path('O0001/', power_robot.O0001, name='O0001'),
    path('O0002/', power_robot.O0002, name='O0002'),
    path('O0003/', power_robot.O0003, name='O0003'),
    path('O0004/', power_robot.O0004, name='O0004'),
    path('O0005/', configuration.O0005, name='O0005'),
    path('O0006/', position_list.O0006, name='O0006'),
    path('O0007/', teach_path.O0007, name='O0007'),
    path('O0008/', move_path.O0008, name='O0008'),
    # path('O0009/', power_robot.O0009, name='O0009'),
    path('O0010/', power_robot.O0010, name='O0010'),
    # path('O0011/', power_robot.O0011, name='O0011'),
    # path('O0012/', power_robot.O0012, name='O0012'),
    path('O0013/', configuration.O0013, name='O0013'),

    path('O0014/', position_list.O0014, name='O0014'),
    path('O0015/', position_list.O0015, name='O0015'),

    path('O0016/', teach_path.O0016, name='O0016'),
    path('O0017/', teach_path.O0017, name='O0017'),

    path('O0021/', move.O0021, name='O0021'),
    path('O0022/', move.O0022, name='O0022'),
    path('O0023/', move.O0023, name='O0023'),
    path('O0024/', move.O0024, name='O0024'),
    path('O0025/', move.O0025, name='O0025'),

    path('O0026/', move_path.O0026, name='O0026'),

    path('O0027/', move.O0027, name='O0027'),
    path('O0028/', move.O0028, name='O0028'),

    path('global/', position_list.global_list, name='global'),
    path('path/', teach_path.path_list, name='path'),
    path('point/', teach_path.point_list, name='point'),

    path('I1001/', components.I1001, name='I1001'),

    #====================================================================================

    path('O0031/', maps.O0031, name='O0031'),
    path('site/', maps.site_list, name='site'),
    path('map/', maps.map_list, name='map'),
    path('save_map/', maps.save_map, name='save_map'),
    path('jog/', move.jog_mode, name='jog'),
    path('missions/', maps.missions, name='missions'),
    path('record/', maps.record, name='record'),
    path('grip/', move_path.grip, name='grip'),
    path('copy/', components.copy, name='copy'),

]
