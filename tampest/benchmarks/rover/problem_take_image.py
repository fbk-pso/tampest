from unified_planning.shortcuts import *
import os
import string

FILE_PATH = os.path.dirname(os.path.abspath(__file__))

class Rover:

    def __init__(self) -> None:
        pass

    def get_problem(self, dim: string, d: int): 

        # types
        if dim == '2D':
            map_file = os.path.join(FILE_PATH, f"configs/resources/2D/{d}rovers.yaml")
        else:
            raise NotImplementedError
        
        occ_map = OccupancyMap(map_file, (0, 0))
        
        Rover = MovableType('rover')
        Door = MovableType('door')
        Waypoint = ConfigurationType('waypoint', occ_map, 3)# UserType('waypoint')
        DoorConfig = ConfigurationType('door_config', occ_map, 3)
        Store = UserType('store')
        Camera = UserType('camera')
        Mode = UserType('mode')
        Lander = UserType('lander')
        #Objective = UserType('objective')

        # fluents
        # at = Fluent("at", BoolType(), r=Rover, w=Waypoint)
        at = Fluent("at", Waypoint, r=Rover)
        #at_lander = Fluent("at_lander", BoolType(), l=Lander, w=Waypoint)
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
        #have_image = Fluent("have_image", BoolType(), rover=Rover, w=Waypoint, m=Mode)
        communicated_soil_data = Fluent("communicated_soil_data", BoolType(), w=Waypoint)
        communicated_rock_data = Fluent("communicated_rock_data", BoolType(), w=Waypoint)
        communicated_image_data = Fluent("communicated_image_data", BoolType(), o=Objective, m=Mode)
        #communicated_image_data = Fluent("communicated_image_data", BoolType(), w=Waypoint, m=Mode)
        at_soil_sample = Fluent("at_soil_sample", BoolType(), w=Waypoint)
        at_rock_sample = Fluent("at_rock_sample", BoolType(), w=Waypoint)
        visible_from = Fluent("visible_from", BoolType(), o=Objective, w=Waypoint)
        #visible_from = Fluent("visible_from", BoolType(), x=Waypoint, y=Waypoint)
        store_of = Fluent("store_of", BoolType(), s=Store, r=Rover)
        calibration_target = Fluent("calibration_target", BoolType(), c=Camera, o=Objective)
        #calibration_target = Fluent("calibration_target", BoolType(), c=Camera, w=Waypoint)
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

        calibrate = InstantaneousAction("calibrate", r=Rover, c=Camera, x=Waypoint, y=Waypoint)
        r_calibrate = calibrate.parameter("r")
        c_calibrate = calibrate.parameter("c")
        x_calibrate = calibrate.parameter("x")
        y_calibrate = calibrate.parameter("y")
        calibrate.add_precondition(equipped_for_imaging(r_calibrate))
        calibrate.add_precondition(calibration_target(c_calibrate, x_calibrate))
        calibrate.add_precondition(Equals(at(r_calibrate), y_calibrate))
        calibrate.add_precondition(visible_from(x_calibrate, y_calibrate))
        calibrate.add_precondition(on_board(c_calibrate, r_calibrate))
        calibrate.add_precondition(Not(calibrated(c_calibrate, r_calibrate)))
        calibrate.add_effect(calibrated(c_calibrate, r_calibrate), True)

        # take_soil_image = InstantaneousAction("take_soil_image", r=Rover, x=Waypoint, y=Waypoint, c=Camera, m=Mode)
        # r_take_soil_image = take_soil_image.parameter("r")
        # x_take_soil_image = take_soil_image.parameter("x")
        # y_take_soil_image = take_soil_image.parameter("y")
        # c_take_soil_image = take_soil_image.parameter("c")
        # m_take_soil_image = take_soil_image.parameter("m")
        # take_soil_image.add_precondition(Equals(at(r_take_soil_image), x_take_soil_image))
        # take_soil_image.add_precondition(at_soil_sample(y_take_soil_image))
        # take_soil_image.add_precondition(calibrated(c_take_soil_image, r_take_soil_image))
        # take_soil_image.add_precondition(on_board(c_take_soil_image, r_take_soil_image))
        # take_soil_image.add_precondition(equipped_for_imaging(r_take_soil_image))
        # take_soil_image.add_precondition(supports(c_take_soil_image, m_take_soil_image))
        # take_soil_image.add_precondition(visible_from(y_take_soil_image, x_take_soil_image))
        # take_soil_image.add_effect(have_image(r_take_soil_image, x_take_soil_image, m_take_soil_image), True)

        # take_rock_image = InstantaneousAction("take_rock_image", r=Rover, x=Waypoint, y=Waypoint, c=Camera, m=Mode)
        # r_take_rock_image = take_rock_image.parameter("r")
        # x_take_rock_image = take_rock_image.parameter("x")
        # y_take_rock_image = take_rock_image.parameter("y")
        # c_take_rock_image = take_rock_image.parameter("c")
        # m_take_rock_image = take_rock_image.parameter("m")
        # take_rock_image.add_precondition(Equals(at(r_take_rock_image), x_take_rock_image))
        # take_rock_image.add_precondition(at_rock_sample(y_take_rock_image))
        # take_rock_image.add_precondition(calibrated(c_take_rock_image, r_take_rock_image))
        # take_rock_image.add_precondition(on_board(c_take_rock_image, r_take_rock_image))
        # take_rock_image.add_precondition(equipped_for_imaging(r_take_rock_image))
        # take_rock_image.add_precondition(supports(c_take_rock_image, m_take_rock_image))
        # take_rock_image.add_precondition(visible_from(y_take_rock_image, x_take_rock_image))
        # take_rock_image.add_effect(have_image(r_take_rock_image, x_take_rock_image, m_take_rock_image), True)

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
        take_image.add_precondition(at(r_take_image, w_take_image))
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

        # communicate_image_data = InstantaneousAction("communicate_image_data", r=Rover, l=Lander, m=Mode, x=Waypoint, y=Waypoint)
        # r_communicate_image_data = communicate_image_data.parameter("r")
        # l_communicate_image_data = communicate_image_data.parameter("l")
        # m_communicate_image_data = communicate_image_data.parameter("m")
        # x_communicate_image_data = communicate_image_data.parameter("x")
        # y_communicate_image_data = communicate_image_data.parameter("y") 
        # communicate_image_data.add_precondition(Equals(at(r_communicate_image_data), x_communicate_image_data))
        # communicate_image_data.add_precondition(Equals(at_lander(l_communicate_image_data), y_communicate_image_data))
        # communicate_image_data.add_precondition(have_image(r_communicate_image_data, x_communicate_image_data, m_communicate_image_data) )
        # communicate_image_data.add_precondition(visible(x_communicate_image_data, y_communicate_image_data))
        # communicate_image_data.add_precondition(available(r_communicate_image_data) )
        # communicate_image_data.add_precondition(channel_free(l_communicate_image_data) )
        # communicate_image_data.add_effect(communicated_image_data(x_communicate_image_data, m_communicate_image_data), True)

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

        robot_footprint = [(-5, 5), (5, 5), (5, -5), (-5, -5)]
        robot_motion_model = MotionModels.REEDSSHEPP
        robot_params = {'turning_radius': 2.0}

        door_footprint = [(-10, 1), (10, 1), (10, -1), (-10, -1)]
        door_motion_model = MotionModels.SE2 
        door_params = {}

        rovers = []
        rover_stores = []
        doors = []
        door_closed_configs = []
        door_open_configs = []
        cameras = []
        starts = []
        soils = [] #{at: ..., visible_from: ...}
        rocks = []

        for i in range(d):
            rovers.append(MovableObject(f'rover{i}', Rover, footprint=robot_footprint, motion_model=robot_motion_model, parameters=robot_params))
            rover_stores.append(Object(f'rover{i}store', Store))
            doors.append(MovableObject(f'door{i}', Door, footprint=door_footprint, motion_model=door_motion_model, parameters=door_params))
            door_closed_configs.append(ConfigurationObject(f'door{i}_closed', DoorConfig, (56.16+i*112.25, 52.0, 0)))
            door_open_configs.append(ConfigurationObject(f'door{i}_open', DoorConfig, (76.16+i*112.25, 52.0, 0)))
            cameras.append(Object(f'camera{i}', Camera))
            starts.append(ConfigurationObject(f'start{i}', Waypoint, (20.0+i*112.25, 120.0, 0)))
        
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

        for i in range(int(d/2)):
            soils.append({'pose': ConfigurationObject(f'soil{i}_pose', Waypoint, (56.16+i*2*112.25, 20.0, 0)), 'image_pose': ConfigurationObject(f'soil{i}_image_pose', Waypoint, (56.16+i*2*112.25, 40.0, 0))}) 
            rocks.append({'pose': ConfigurationObject(f'rock{i}_pose', Waypoint, (168.4+i*2*112.25, 20.0, 0)), 'image_pose': ConfigurationObject(f'rock{i}_image_pose', Waypoint, (168.4+i*2*112.25, 40.0, 0))}) 

        l0 = ConfigurationObject('lander0', Waypoint, (d*112.25/2, 158.7, 0))
        
        problem.add_objects(rovers)
        problem.add_objects(rover_stores)
        problem.add_objects(doors)
        problem.add_objects(door_closed_configs)
        problem.add_objects(door_open_configs)
        problem.add_objects(cameras)
        problem.add_objects(starts)

        for s in soils:
            problem.add_objects([s["pose"], s["image_pose"]])

        for r in rocks:
            problem.add_objects([r["pose"], r["image_pose"]])

        problem.add_objects([global_lander, colour, high_res, low_res, l0])

        #problem.add_actions([navigate, open_door, sample_soil, sample_rock, drop, calibrate, take_soil_image, take_rock_image, communicate_soil_data, communicate_rock_data, communicate_image_data])
        problem.add_actions([navigate, open_door, sample_soil, sample_rock, drop, calibrate, take_image, communicate_soil_data, communicate_rock_data, communicate_image_data])

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
                
                problem.set_initial_value(visible(starts[i*2], soils[i]["pose"]), True) 
                problem.set_initial_value(visible(soils[i]["pose"], starts[i*2]), True)

                problem.set_initial_value(visible(starts[i*2], soils[i]["image_pose"]), True) 
                problem.set_initial_value(visible(soils[i]["image_pose"], starts[i*2]), True)

                problem.set_initial_value(visible(starts[i*2+1], rocks[i]["pose"]), True) 
                problem.set_initial_value(visible(rocks[i]["pose"], starts[i*2+1]), True) 

                problem.set_initial_value(visible(starts[i*2+1], rocks[i]["image_pose"]), True) 
                problem.set_initial_value(visible(rocks[i]["image_pose"], starts[i*2+1]), True) 

                for s in soils:
                    problem.set_initial_value(calibration_target(cameras[i*2], s["pose"]), True) 

                for r in rocks:
                    problem.set_initial_value(calibration_target(cameras[i*2+1], r["pose"]), True)

            problem.set_initial_value(equipped_for_imaging(rovers[i]), True)
            problem.set_initial_value(on_board(cameras[i], rovers[i]), True)

            problem.set_initial_value(supports(cameras[i], colour), True)
            problem.set_initial_value(supports(cameras[i], high_res), True)

            problem.set_initial_value(visible(starts[i], l0), True)

        for i in range(int(d/2)):
            problem.set_initial_value(at_soil_sample(soils[i]["pose"]), True)
            problem.set_initial_value(at_rock_sample(rocks[i]["pose"]), True)

            problem.add_goal(communicated_soil_data(soils[i]["pose"]))
            problem.add_goal(communicated_rock_data(rocks[i]["pose"]))

            problem.set_initial_value(visible_from(soils[i]["pose"], soils[i]["image_pose"]), True)
            problem.set_initial_value(visible_from(rocks[i]["pose"], rocks[i]["image_pose"]), True)

            problem.set_initial_value(visible(soils[i]["pose"], soils[i]["image_pose"]), True)
            problem.set_initial_value(visible(soils[i]["image_pose"], soils[i]["pose"]), True)

            problem.set_initial_value(visible(rocks[i]["pose"], rocks[i]["image_pose"]), True)
            problem.set_initial_value(visible(rocks[i]["image_pose"], rocks[i]["pose"]), True)

        for s in soils:
            problem.set_initial_value(visible(s["pose"], l0), True)
            problem.set_initial_value(visible(s["image_pose"], l0), True)
            problem.add_goal(communicated_image_data(s["image_pose"], high_res))

        for r in rocks:
            problem.set_initial_value(visible(r["pose"], l0), True)
            problem.set_initial_value(visible(r["image_pose"], l0), True)
            problem.add_goal(communicated_image_data(r["image_pose"], high_res))

        #problem.add_goal(Equals(at(rovers[2]), soils[1]["pose"]))
        #problem.add_goal(Equals(at(rovers[3]), rocks[1]["pose"]))
        return problem
