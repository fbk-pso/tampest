from unified_planning.shortcuts import *
import os
import string

FILE_PATH = os.path.dirname(os.path.abspath(__file__))

class Rover:

    def __init__(self) -> None:
        pass

    def get_problem(self, dim: string, d: int, c: int): 

        # types
        map_file = os.path.join(FILE_PATH, f"configs/resources/{dim}/{d}rovers.yaml")
        
        if c> 4:
            raise NotImplementedError
        
        occ_map = OccupancyMap(map_file, (0, 0))
        
        Rover = MovableType('rover')
        Door = MovableType('door')
        Waypoint = ConfigurationType('waypoint', occ_map, 3)
        DoorConfig = ConfigurationType('door_config', occ_map, 3)
        Store = UserType('store')
        Camera = UserType('camera')
        Mode = UserType('mode')
        Lander = UserType('lander')
        Objective = UserType('objective')

        # fluents
        at = Fluent("at", Waypoint, r=Rover)
        at_lander = Fluent("at_lander", Waypoint, l=Lander)
        equipped_for_soil_analysis = Fluent("equipped_for_soil_analysis", BoolType(), r=Rover)
        equipped_for_rock_analysis = Fluent("equipped_for_rock_analysis", BoolType(), r=Rover)
        equipped_for_imaging = Fluent("equipped_for_imaging", BoolType(), r=Rover)
        empty = Fluent("empty", BoolType(), s=Store)
        have_rock_analysis = Fluent("have_rock_analysis", BoolType(), r=Rover, w=Waypoint)
        have_soil_analysis = Fluent("have_soil_analysis", BoolType(), r=Rover, w=Waypoint)
        full = Fluent("full", BoolType(), s=Store)
        calibrated = Fluent("calibrated", BoolType(), c=Camera, r=Rover)
        supports = Fluent("supports", BoolType(), c=Camera, m=Mode)
        available = Fluent("available", BoolType(), r=Rover)
        visible = Fluent("visible", BoolType(), x=Waypoint, y=Waypoint)
        have_image = Fluent("have_image", BoolType(), rover=Rover, o=Objective, m=Mode)
        communicated_soil_data = Fluent("communicated_soil_data", BoolType(), w=Waypoint)
        communicated_rock_data = Fluent("communicated_rock_data", BoolType(), w=Waypoint)
        communicated_image_data = Fluent("communicated_image_data", BoolType(), o=Objective, m=Mode)
        at_soil_sample = Fluent("at_soil_sample", BoolType(), w=Waypoint)
        at_rock_sample = Fluent("at_rock_sample", BoolType(), w=Waypoint)
        visible_from = Fluent("visible_from", BoolType(), o=Objective, w=Waypoint)
        store_of = Fluent("store_of", BoolType(), s=Store, r=Rover)
        calibration_target = Fluent("calibration_target", BoolType(), c=Camera, o=Objective)
        on_board = Fluent("on_board", BoolType(), c=Camera, r=Rover)
        channel_free = Fluent("channel_free", BoolType(), l=Lander)

        door_at = Fluent('door_at', DoorConfig, door= Door)
        opened = Fluent('open_config', DoorConfig, door= Door)
        closed = Fluent('close_config', DoorConfig, door= Door)

        sample_soil = InstantaneousAction("sample_soil", r=Rover, s=Store, w=Waypoint)
        r_sample_soil = sample_soil.parameter("r")
        s_sample_soil = sample_soil.parameter("s")
        w_sample_soil = sample_soil.parameter("w")
        sample_soil.add_precondition(Equals(at(r_sample_soil), w_sample_soil))
        sample_soil.add_precondition(at_soil_sample(w_sample_soil))
        sample_soil.add_precondition(equipped_for_soil_analysis(r_sample_soil))
        sample_soil.add_precondition(store_of(s_sample_soil, r_sample_soil))
        sample_soil.add_precondition(empty(s_sample_soil))
        sample_soil.add_effect(empty(s_sample_soil) , False)
        sample_soil.add_effect(full(s_sample_soil), True)
        sample_soil.add_effect(have_soil_analysis(r_sample_soil, w_sample_soil), True)
        sample_soil.add_effect(at_soil_sample(w_sample_soil), False)

        sample_rock = InstantaneousAction("sample_rock", r=Rover, s=Store, w=Waypoint)
        r_sample_rock = sample_rock.parameter("r")
        s_sample_rock = sample_rock.parameter("s")
        w_sample_rock = sample_rock.parameter("w")
        sample_rock.add_precondition(Equals(at(r_sample_rock), w_sample_rock))
        sample_rock.add_precondition(at_rock_sample(w_sample_rock))
        sample_rock.add_precondition(equipped_for_rock_analysis(r_sample_rock))
        sample_rock.add_precondition(store_of(s_sample_rock, r_sample_rock))
        sample_rock.add_precondition(empty(s_sample_rock))
        sample_rock.add_effect(empty(s_sample_rock), False)
        sample_rock.add_effect(full(s_sample_rock), True)
        sample_rock.add_effect(have_rock_analysis(r_sample_rock, w_sample_rock), True)
        sample_rock.add_effect(at_rock_sample(w_sample_rock), False)

        drop = InstantaneousAction("drop", r=Rover, s=Store)
        r_drop = drop.parameter("r")
        s_drop = drop.parameter("s")
        drop.add_precondition(store_of(s_drop, r_drop))
        drop.add_precondition(full(s_drop))
        drop.add_effect(full(s_drop), False)
        drop.add_effect(empty(s_drop), True)

        calibrate = InstantaneousAction("calibrate", r=Rover, c=Camera, o=Objective, y=Waypoint)
        r_calibrate = calibrate.parameter("r")
        c_calibrate = calibrate.parameter("c")
        o_calibrate = calibrate.parameter("o")
        y_calibrate = calibrate.parameter("y")
        calibrate.add_precondition(equipped_for_imaging(r_calibrate))
        calibrate.add_precondition(calibration_target(c_calibrate, o_calibrate))
        calibrate.add_precondition(Equals(at(r_calibrate), y_calibrate))
        calibrate.add_precondition(visible_from(o_calibrate, y_calibrate))
        calibrate.add_precondition(on_board(c_calibrate, r_calibrate))
        calibrate.add_precondition(Not(calibrated(c_calibrate, r_calibrate)))
        calibrate.add_effect(calibrated(c_calibrate, r_calibrate), True)

        take_image = InstantaneousAction("take_image", r=Rover, w=Waypoint, o=Objective, c=Camera, m=Mode)
        r_take_image = take_image.parameter("r")
        w_take_image = take_image.parameter("w")
        o_take_image = take_image.parameter("o")
        c_take_image = take_image.parameter("c")
        m_take_image = take_image.parameter("m")
        take_image.add_precondition(calibrated(c_take_image, r_take_image))
        take_image.add_precondition(on_board(c_take_image, r_take_image))
        take_image.add_precondition(equipped_for_imaging(r_take_image))
        take_image.add_precondition(supports(c_take_image, m_take_image))
        take_image.add_precondition(visible_from(o_take_image, w_take_image))
        take_image.add_precondition(Equals(at(r_take_image), w_take_image))
        take_image.add_effect(have_image(r_take_image, o_take_image, m_take_image), True)
        take_image.add_effect(calibrated(c_take_image, r_take_image), False)

        communicate_soil_data = InstantaneousAction("communicate_soil_data", r=Rover, l=Lander, x=Waypoint, y=Waypoint, z=Waypoint)
        r_communicated_soil_data = communicate_soil_data.parameter("r")
        l_communicated_soil_data = communicate_soil_data.parameter("l")
        x_communicated_soil_data = communicate_soil_data.parameter("x")
        y_communicated_soil_data = communicate_soil_data.parameter("y")
        z_communicated_soil_data = communicate_soil_data.parameter("z")
        communicate_soil_data.add_precondition(Equals(at(r_communicated_soil_data), y_communicated_soil_data))
        communicate_soil_data.add_precondition(Equals(at_lander(l_communicated_soil_data), z_communicated_soil_data))
        communicate_soil_data.add_precondition(have_soil_analysis(r_communicated_soil_data, x_communicated_soil_data))
        communicate_soil_data.add_precondition(visible(y_communicated_soil_data, z_communicated_soil_data) )
        communicate_soil_data.add_precondition(available(r_communicated_soil_data))
        communicate_soil_data.add_precondition(channel_free(l_communicated_soil_data))
        communicate_soil_data.add_effect(communicated_soil_data(x_communicated_soil_data), True)

        communicate_rock_data = InstantaneousAction("communicate_rock_data",  r=Rover, l=Lander, x=Waypoint, y=Waypoint, z=Waypoint)
        r_communicate_rock_data = communicate_rock_data.parameter("r")
        l_communicate_rock_data = communicate_rock_data.parameter("l")
        x_communicate_rock_data = communicate_rock_data.parameter("x")
        y_communicated_rock_data = communicate_rock_data.parameter("y")
        z_communicated_rock_data = communicate_rock_data.parameter("z")
        communicate_rock_data.add_precondition(Equals(at(r_communicate_rock_data), y_communicated_rock_data))
        communicate_rock_data.add_precondition(Equals(at_lander(l_communicate_rock_data), z_communicated_rock_data))
        communicate_rock_data.add_precondition(have_rock_analysis(r_communicate_rock_data, x_communicate_rock_data))
        communicate_rock_data.add_precondition(visible(y_communicated_rock_data, z_communicated_rock_data))
        communicate_rock_data.add_precondition(available(r_communicate_rock_data))
        communicate_rock_data.add_precondition(channel_free(l_communicate_rock_data))
        communicate_rock_data.add_effect(communicated_rock_data(x_communicate_rock_data), True)

        communicate_image_data = InstantaneousAction("communicate_image_data", r=Rover, l=Lander, o= Objective, m=Mode, x=Waypoint, y=Waypoint)
        r_communicate_image_data = communicate_image_data.parameter("r")
        l_communicate_image_data = communicate_image_data.parameter("l")
        o_communicate_image_data = communicate_image_data.parameter("o")
        m_communicate_image_data = communicate_image_data.parameter("m")
        x_communicate_image_data = communicate_image_data.parameter("x")
        y_communicate_image_data = communicate_image_data.parameter("y") 
        communicate_image_data.add_precondition(Equals(at(r_communicate_image_data), x_communicate_image_data))
        communicate_image_data.add_precondition(Equals(at_lander(l_communicate_image_data), y_communicate_image_data))
        communicate_image_data.add_precondition(have_image(r_communicate_image_data, o_communicate_image_data, m_communicate_image_data) )
        communicate_image_data.add_precondition(visible(x_communicate_image_data, y_communicate_image_data))
        communicate_image_data.add_precondition(available(r_communicate_image_data) )
        communicate_image_data.add_precondition(channel_free(l_communicate_image_data) )
        communicate_image_data.add_effect(communicated_image_data(o_communicate_image_data, m_communicate_image_data), True)
                                                                    
        open_door = InstantaneousMotionAction(f'open', door=Door, c_close=DoorConfig, c_open=DoorConfig)
        d_open = open_door.parameter("door")
        c_close = open_door.parameter("c_close")
        c_open = open_door.parameter("c_open")
        open_door.add_precondition(Equals(door_at(d_open), c_close))
        open_door.add_precondition(Equals(closed(d_open), c_close))
        open_door.add_precondition(Equals(opened(d_open), c_open))
        open_door.add_effect(door_at(d_open), c_open)
        open_door.add_motion_constraint(Waypoints(d_open, c_close, [c_open]))

        # problem
        problem = Problem("rover")

        global_lander = Object("global_lander", Lander)

        colour = Object("colour", Mode)
        high_res = Object("high_res", Mode)
        low_res = Object("low_res", Mode)

        if dim == "2D":
            robot_footprint = [(-5, 5), (5, 5), (5, -5), (-5, -5)]
            robot_model = None
            robot_motion_model = MotionModels.REEDSSHEPP
            robot_params = {'turning_radius': 2.0}

            door_footprint = [(-10, 1), (10, 1), (10, -1), (-10, -1)]
            door_model = None
            door_motion_model = MotionModels.SE2 
            door_params = {}

        if dim == "3D":
            robot_footprint = None
            robot_model = os.path.join(FILE_PATH, f"configs/resources/3D/robot.dae")
            robot_motion_model = MotionModels.SE3
            robot_params = {}

            door_footprint = None
            door_model = os.path.join(FILE_PATH, f"configs/resources/3D/door.dae")
            door_motion_model = MotionModels.SE3 
            door_params = {}

        rovers = []
        rover_stores = []
        doors = []
        door_closed_configs = []
        door_open_configs = []
        cameras = []
        starts = []
        soils = [] 
        rocks = []
        x_images= {}
        y_images= {}
        x_objectives = {}
        y_objectives = {}

        for i in range(d):
            rovers.append(MovableObject(f'rover{i}', Rover, footprint=robot_footprint, model=robot_model, motion_model=robot_motion_model, parameters=robot_params))
            rover_stores.append(Object(f'rover{i}store', Store))
            doors.append(MovableObject(f'door{i}', Door, footprint=door_footprint, model = door_model, motion_model=door_motion_model, parameters=door_params))
            cameras.append(Object(f'camera{i}', Camera))
            
            if dim == "2D":
                door_closed_configs.append(ConfigurationObject(f'door{i}_closed', DoorConfig, (56.16+i*112.25, 52.0, 0)))
                door_open_configs.append(ConfigurationObject(f'door{i}_open', DoorConfig, (76.16+i*112.25, 52.0, 0))) 
                starts.append(ConfigurationObject(f'start{i}', Waypoint, (20.0+i*112.25, 120.0, 0)))
            
        for i in range(int(d/2)):
            if dim == "3D":
                starts.append(ConfigurationObject(f'start{2*i}', Waypoint, (-30.0-i*37, 10.0, 0, 0, 0, 0, 0)))
                door_closed_configs.append(ConfigurationObject(f'door{2*i}_closed', DoorConfig, (-18.0-37.0*i, -13.0, 0, 0, 0, 1, 1.57)))
                door_open_configs.append(ConfigurationObject(f'door{2*i}_open', DoorConfig, (-10.0-37*i, -13.0, 0, 0, 0, 1, 1.57)))     

                starts.append(ConfigurationObject(f'start{2*i+1}', Waypoint, (30.0 + i*37, 10.0, 0, 0, 0, 0, 0)))
                door_closed_configs.append(ConfigurationObject(f'door{2*i+1}_closed', DoorConfig, (18.0+37*i, -13.0, 0, 0, 0, 1, 1.57)))
                door_open_configs.append(ConfigurationObject(f'door{2*i+1}_open', DoorConfig, (10.0+37*i, -13.0, 0, 0, 0, 1, 1.57))) 
                    
        
        navigate = InstantaneousMotionAction("navigate", r=Rover, x=Waypoint, y=Waypoint)
        r_navigate = navigate.parameter("r")
        x_navigate = navigate.parameter("x")
        y_navigate = navigate.parameter("y")
        navigate.add_precondition(available(r_navigate))
        navigate.add_precondition(Not(Equals(x_navigate, y_navigate)))
        navigate.add_precondition(Equals(at(r_navigate), x_navigate))
        navigate.add_precondition(visible(x_navigate, y_navigate))
        navigate.add_effect(at(r_navigate), y_navigate)
        navigate.add_motion_constraint(Waypoints(r_navigate, x_navigate, [y_navigate], {door: door_at(door) for door in doors}))

        all_x_images = {}
        all_y_images = {}
        for i in range(int(d/2)):
            if dim == "2D":
                soil_pose = (56.16+i*2*112.25, 20.0, 0)
                rock_pose = (168.4+i*2*112.25, 20.0, 0) 
            if dim=="3D":
                soil_pose = (-18-37*i, -23.0, 0, 0, 0, 0, 0)
                rock_pose = (18+37*i, -23.0, 0, 0, 0, 0, 0)              
            
            soils.append(ConfigurationObject(f'soil{i}_pose', Waypoint, soil_pose)) 
            rocks.append(ConfigurationObject(f'rock{i}_pose', Waypoint, rock_pose)) 

            if c>0:
                x_objectives[i] = [Object(f'x_obj{i}{j}', Objective) for j in range(c) ]
                y_objectives[i] = [Object(f'y_obj{i}{j}', Objective) for j in range(c) ]

                if dim == "2D":

                    all_x_images[i] = [ConfigurationObject(f'x_img{i}{0}_pose', Waypoint, (soil_pose[0] - 10.0, soil_pose[1] + 10.0, 0)),
                                       ConfigurationObject(f'x_img{i}{1}_pose', Waypoint, (soil_pose[0] - 10.0, soil_pose[1] - 10.0, 0)), 
                                       ConfigurationObject(f'x_img{i}{2}_pose', Waypoint, (soil_pose[0] + 10.0, soil_pose[1] + 10.0, 0)),
                                       ConfigurationObject(f'x_img{i}{3}_pose', Waypoint, (soil_pose[0] + 10.0, soil_pose[1] - 10.0, 0))]

                    all_y_images[i] = [ConfigurationObject(f'y_img{i}{0}_pose', Waypoint, (rock_pose[0] - 10.0, rock_pose[1] + 10.0, 0)),
                                       ConfigurationObject(f'y_img{i}{1}_pose', Waypoint, (rock_pose[0] - 10.0, rock_pose[1] - 10.0, 0)),
                                       ConfigurationObject(f'y_img{i}{2}_pose', Waypoint, (rock_pose[0] + 10.0, rock_pose[1] + 10.0, 0)),
                                       ConfigurationObject(f'y_img{i}{3}_pose', Waypoint, (rock_pose[0] + 10.0, rock_pose[1] - 10.0, 0))]
            
                if dim=="3D":

                    all_x_images[i] = [ConfigurationObject(f'x_img{i}{0}_pose', Waypoint, (soil_pose[0] - 7.0, soil_pose[1] + 3.0, 0, 0, 0, 0, 0)), # top left
                                       ConfigurationObject(f'x_img{i}{1}_pose', Waypoint, (soil_pose[0] - 7.0, soil_pose[1] - 3.0, 0, 0, 0, 0, 0)), # bottom left
                                       ConfigurationObject(f'x_img{i}{2}_pose', Waypoint, (soil_pose[0] + 7.0, soil_pose[1] + 3.0, 0, 0, 0, 0, 0)), # top right
                                       ConfigurationObject(f'x_img{i}{3}_pose', Waypoint, (soil_pose[0] + 7.0, soil_pose[1] - 3.0, 0, 0, 0, 0, 0))] # bottom right
                    
                    all_y_images[i] = [ConfigurationObject(f'y_img{i}{0}_pose', Waypoint, (rock_pose[0] - 7.0, rock_pose[1] + 3.0, 0, 0, 0, 0, 0)),
                                   ConfigurationObject(f'y_img{i}{1}_pose', Waypoint, (rock_pose[0] - 7.0, rock_pose[1] - 3.0, 0, 0, 0, 0, 0)),
                                   ConfigurationObject(f'y_img{i}{2}_pose', Waypoint, (rock_pose[0] + 7.0, rock_pose[1] + 3.0, 0, 0, 0, 0, 0)),
                                   ConfigurationObject(f'y_img{i}{3}_pose', Waypoint, (rock_pose[0] + 7.0, rock_pose[1] - 3.0, 0, 0, 0, 0, 0))]
                
                x_images[i] = all_x_images[i][:c]
                y_images[i] = all_y_images[i][:c]
        
        if dim == "2D":
            l0 = ConfigurationObject('lander0', Waypoint, (d*112.25/2, 158.7, 0))

        if dim == "3D":
            l0 = ConfigurationObject('lander0', Waypoint, (0, 25, 0, 0, 0, 0, 0))
        
        problem.add_objects(rovers)
        problem.add_objects(rover_stores)
        problem.add_objects(doors)
        problem.add_objects(door_closed_configs)
        problem.add_objects(door_open_configs)
        problem.add_objects(cameras)
        problem.add_objects(starts)

        problem.add_objects(soils)
        problem.add_objects(rocks)

        problem.add_objects([global_lander, colour, high_res, low_res, l0])

        #problem.add_actions([navigate, open_door, sample_soil, sample_rock, drop, calibrate, take_soil_image, take_rock_image, communicate_soil_data, communicate_rock_data, communicate_image_data])
        if c>0:
            problem.add_actions([navigate, open_door, sample_soil, sample_rock, drop, calibrate, take_image, communicate_soil_data, communicate_rock_data, communicate_image_data])
            problem.add_objects([value for values in x_images.values() for value in values])
            problem.add_objects([value for values in y_images.values() for value in values])
            problem.add_objects([value for values in x_objectives.values() for value in values])
            problem.add_objects([value for values in y_objectives.values() for value in values])
        else:
            problem.add_actions([navigate, open_door, sample_soil, sample_rock, drop, communicate_soil_data, communicate_rock_data])

        problem.add_fluents([at, at_lander, door_at, opened, closed])
        problem.add_fluent(equipped_for_soil_analysis, default_initial_value=False)
        problem.add_fluent(equipped_for_rock_analysis, default_initial_value=False)
        problem.add_fluent(equipped_for_imaging, default_initial_value=False)
        problem.add_fluent(empty, default_initial_value=False)
        problem.add_fluent(have_rock_analysis, default_initial_value=False)
        problem.add_fluent(have_soil_analysis, default_initial_value=False)
        problem.add_fluent(full, default_initial_value=False)
        problem.add_fluent(calibrated, default_initial_value=False)
        problem.add_fluent(supports, default_initial_value=False)
        problem.add_fluent(available, default_initial_value=False)
        problem.add_fluent(visible, default_initial_value=False)
        problem.add_fluent(have_image, default_initial_value=False)
        problem.add_fluent(communicated_soil_data, default_initial_value=False)
        problem.add_fluent(communicated_rock_data, default_initial_value=False)
        problem.add_fluent(communicated_image_data, default_initial_value=False)
        problem.add_fluent(at_soil_sample, default_initial_value=False)
        problem.add_fluent(at_rock_sample, default_initial_value=False)
        problem.add_fluent(visible_from, default_initial_value=False)
        problem.add_fluent(store_of, default_initial_value=False)
        problem.add_fluent(calibration_target, default_initial_value=False)
        problem.add_fluent(on_board, default_initial_value=False)
        problem.add_fluent(channel_free, default_initial_value=False)
   
        problem.set_initial_value(at_lander(global_lander), l0)
        problem.set_initial_value(channel_free(global_lander), True)

        for i in range(d):
            problem.set_initial_value(opened(doors[i]), door_open_configs[i])
            problem.set_initial_value(closed(doors[i]), door_closed_configs[i])
            problem.set_initial_value(door_at(doors[i]), door_closed_configs[i])

            problem.set_initial_value(at(rovers[i]), starts[i])
            problem.set_initial_value(available(rovers[i]), True)
            problem.set_initial_value(store_of(rover_stores[i], rovers[i]), True)
            problem.set_initial_value(empty(rover_stores[i]), True)

            if i*2+1 < len(rovers):
                problem.set_initial_value(equipped_for_soil_analysis(rovers[i*2]), True) 
                problem.set_initial_value(equipped_for_rock_analysis(rovers[i*2+1]), True) 
                
                problem.set_initial_value(visible(starts[i*2], soils[i]), True) 
                problem.set_initial_value(visible(soils[i], starts[i*2]), True)

                if x_images and x_objectives:
                    for x_im in x_images[i]:
                        problem.set_initial_value(visible(starts[i*2], x_im), True)
                        problem.set_initial_value(visible(x_im, starts[i*2]), True)
                    for x_o in x_objectives[i]:
                        problem.set_initial_value(calibration_target(cameras[i*2], x_o), True) 
                        for x_im in x_images[i]:
                            problem.set_initial_value(visible_from(x_o, x_im), True)

                problem.set_initial_value(visible(starts[i*2+1], rocks[i]), True) 
                problem.set_initial_value(visible(rocks[i], starts[i*2+1]), True) 

                if y_images and y_objectives:
                    for y_im in y_images[i]:
                        problem.set_initial_value(visible(starts[i*2+1], y_im), True)
                        problem.set_initial_value(visible(y_im, starts[i*2+1]), True)
                
                    for y_o in y_objectives[i]:
                        problem.set_initial_value(calibration_target(cameras[i*2+1], y_o), True)
                        for y_im in y_images[i]:
                            problem.set_initial_value(visible_from(y_o, y_im), True)

            problem.set_initial_value(equipped_for_imaging(rovers[i]), True)
            problem.set_initial_value(on_board(cameras[i], rovers[i]), True)

            problem.set_initial_value(supports(cameras[i], colour), True)
            problem.set_initial_value(supports(cameras[i], high_res), True)

            problem.set_initial_value(visible(starts[i], l0), True)

        for i in range(int(d/2)):
            problem.set_initial_value(at_soil_sample(soils[i]), True)
            problem.set_initial_value(at_rock_sample(rocks[i]), True)

            problem.add_goal(communicated_soil_data(soils[i]))
            problem.add_goal(communicated_rock_data(rocks[i]))

            if x_images:
                for x in x_images[i]:
                    problem.set_initial_value(visible(soils[i], x), True)
                    problem.set_initial_value(visible(x, soils[i]), True)

            if y_images:
                for y in y_images[i]:
                    problem.set_initial_value(visible(rocks[i], y), True)
                    problem.set_initial_value(visible(y, rocks[i]), True)

        for s in soils:
            problem.set_initial_value(visible(s, l0), True)

        for r in rocks:
            problem.set_initial_value(visible(r, l0), True)
            
        for _, x_objs in x_objectives.items():
            for o in x_objs:
                problem.add_goal(communicated_image_data(o, high_res))

        for _, y_objs in y_objectives.items():
            for o in y_objs:
                problem.add_goal(communicated_image_data(o, high_res))


        #problem.add_goal(Equals(door_at(doors[0]), door_open_configs[0]))
        #problem.add_goal(Equals(at(rovers[2]), soils[1]))
        #problem.add_goal(Equals(at(rovers[5]), rocks[2]))
        return problem
