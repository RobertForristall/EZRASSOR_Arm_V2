import rclpy
import ezrassor_arm_v2.utility_functions as uf
import ezrassor_arm_v2.nav_functions as nf

from time import sleep

def at_target(current_x, current_y, target_x, target_y, theshold):

    value = (
        (target_x - theshold) < current_x < (target_x + theshold)
        and (target_y - theshold) < current_y < (target_y + theshold)
    )

    return value

def charge_battery(world_state, ros_util):

    ros_util.publish_actions(uf.actions)

    while world_state.battery < 100:
        sleep(0.1)
        world_state.battery += 3
    
    world_state.battery = 100

def auto_drive_location(world_state, ros_util, waypoint_server=None):

    if waypoint_server is None:
        world_state.logger.info(
            'Auto-driving to [{}, {}]'.format(
                world_state.target_location.x,
                world_state.target_location.y
            )
        )
    
    preempted = False
    #feedback = uf.send_feedback(world_state, waypoint_server)

    world_state.logger.info(
        'Raising drums before driving...'
    )

    if ros_util.rover_model == 'arm':
        uf.set_back_arm_angle(world_state, ros_util, 0.5)
    else:
        uf.set_front_arm_angle(world_state, ros_util, 1.3)
        uf.set_back_arm_angle(world_state, ros_util, 1.3)

    world_state.logger.info(
        'Drums raised, performing self check...'
    )

    if uf.self_check(world_state, ros_util) != 1:
        preempted = True

        if waypoint_server is not None:
            waypoint_server.set_preempted()
        
        world_state.logger.info(
            'Status Check Failed!'
        )

        #return feedback, preempted
        return preempted
    
    world_state.logger.info(
        'Self check complete, getting new heading...'
    )
    
    new_heading_deg = nf.calculate_heading(world_state)
    angle2goal_rad = nf.adjust_angle(
        world_state.heading, new_heading_deg
    )

    if angle2goal_rad < 0:
        direction = 'right'
    else:
        direction = 'left'
    
    world_state.logger.info(
        'New heading obtained, turning to face target...'
    )

    uf.turn(
        new_heading_deg, 
        direction, 
        world_state, 
        ros_util
    )

    ros_util.publish_actions(uf.actions)

    world_state.logger.info(
        'Turn complete, begining movement loop...'
    )

    while not at_target(
        world_state.position['x'],
        world_state.position['y'],
        world_state.target_location.x,
        world_state.target_location.y,
        ros_util.threshold
    ):
        if (
            waypoint_server is not None
            and waypoint_server.is_preempt_requested()
        ):
            preempted = True
            break
        
        if ros_util.rover_model == 'arm':
            uf.set_back_arm_angle(world_state, ros_util, 0.5)
        else:
            uf.set_front_arm_angle(world_state, ros_util, 1.3)
            uf.set_back_arm_angle(world_state, ros_util, 1.3)

        if uf.self_check(world_state, ros_util) != 1:
            preempted = True

            if waypoint_server is not None:
                waypoint_server.set_preempted()
            
            world_state.logger.info(
                'Status Check Failed!'
            )
            break
            
        angle = uf.get_turn_angle(world_state, ros_util)

        direction = 'right' if angle < 0 else 'left'

        uf.turn(
            nf.rel_to_abs(world_state.heading, angle),
            direction,
            world_state,
            ros_util
        )

        uf.move(ros_util.move_increment, world_state, ros_util)

        world_state.battery -= 0.1

        #feedback = uf.send_feedback(world_state, waypoint_server)
    
    if waypoint_server is None:
        world_state.logger.info(
            'Destination reached!'
        )
    
    ros_util.publish_actions(uf.actions)
    #return feedback, preempted
    return preempted

def auto_dig(world_state, ros_util, duration, waypoint_server=None):

    #feedback = uf.send_feedback(world_state, waypoint_server)
    preempted = False

    if uf.self_check(world_state, ros_util) != 1:

        if waypoint_server is not None:
            waypoint_server.set_preempted()
        
        preempted = True

        #return feedback, preempted
        return preempted
    
    if waypoint_server is None:
        world_state.logger.info(
            'Auto-digging for {} seconds'.format(
                duration
            )
        )
    
    if ros_util.rover_model == 'arm':
        uf.set_back_arm_angle(world_state, ros_util, -0.1)
    else:
        uf.set_front_arm_angle(world_state, ros_util, -0.1)
        uf.set_back_arm_angle(world_state, ros_util, -0.1)
    
    t = 0
    direction = 'forward'
    while t < duration:

        if t % 100 == 0:
            direction = 'reverse' if direction == 'forward' else 'forward'
        
        temp_actions = uf.actions
        if ros_util.rover_model == 'arm':
            temp_actions.update({'back_drum': 1.0})
        else:
            temp_actions.update({'front_drum': 1.0, 'back_drum': 1.0})
        
        ros_util.publish_actions(temp_actions)

        t += 1
        ros_util.rate.sleep()

        world_state.battery -= 0.5

        #feedback = uf.send_feedback(world_state, waypoint_server)

        if (
            waypoint_server is not None
            and waypoint_server.is_preempt_requested()
        ):
            preempted = True
            break

        if uf.self_check(world_state, ros_util) != 1:
            if waypoint_server is not None:
                waypoint_server.set_preempted()
            
            preempted = True
            break
            
    ros_util.publish_actions(uf.actions)
    #return feedback, preempted
    return preempted

def auto_dock(world_state, ros_util):

    world_state.logger.info(
        'Auto-returning to origin...'
    )

    old_target = world_state.target_location

    ros_util.threshold = 3

    world_state.target_location.x = 0
    world_state.target_location.y = 0

    auto_drive_location(world_state, ros_util)

    ros_util.threshold = 0.5

    world_state.target_location = old_target

def auto_dump(world_state, ros_util, duration):

    temp_actions = uf.actions

    world_state.logger.info(
        'Auto-dumping drum contents'
    )

    if ros_util.rover_model == 'arm':
        uf.set_back_arm_angle(world_state, ros_util, 0.5)
    else:
        uf.set_front_arm_angle(world_state, ros_util, 1.3)
        uf.set_back_arm_angle(world_state, ros_util, 1.3)
    
    t = 0
    while t < duration * 40:
        if uf.self_check(world_state, ros_util) != 1:
            return

        if ros_util.rover_model == 'arm':
            temp_actions.update({'back_drum': -1.0})
        else:
            temp_actions.update({'front_drum': -1.0, 'back_drum': -1.0})
        ros_util.publish_actions(temp_actions)
        t += 1
        ros_util.rate.sleep()

    new_heading = world_state.heading = (world_state.heading + 180) % 360

    while not (
        (new_heading - 1) < world_state.heading < (new_heading + 1)
    ):
        temp_actions.update({
            'front_drum': 0, 
            'back_drum': 0, 
            'movement': 'left'
        })

        ros_util.publish_actions(temp_actions)
        ros_util.rate.sleep()
    
    while t < duration * 30:

        if ros_util.rover_model == 'arm':
            temp_actions.update({
                'back_drum': -1.0, 
                'movement': 'stop'
            })
        else:
            temp_actions.update({
                'front_drum': -1.0, 
                'back_drum': -1.0, 
                'movement': 'stop'
            })
        
        ros_util.publish_actions(temp_actions)
        t += 1
        ros_util.rate.sleep()
    
    ros_util.publish_actions(uf.actions)

