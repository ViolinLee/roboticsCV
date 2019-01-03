import numpy as np
import random


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # if predicted naving area isn't maped --> forward
                if not predicted_area(Rover):
                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle 
                    if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    
                # if predicted area is maped,
                # steer --> goal
                else:
                    if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # set steering to goal
                    Rover.steer = steer_foward(Rover)
 
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    if len(Rover.nav_angles) > Rover.go_forward/2:
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)# Could be more clever here about which way to turn
                    else:
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        Rover.steer = random.randrange(-1, 2, 2) * 15
                        
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = steer_stop(Rover) 
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

def steer_stop(Rover):

    if rover_goal(Rover) == 'left' and (len(Rover.nav_angles_left) > (Rover.go_forward / 2)):
        steer = np.clip(np.mean(Rover.nav_angles_left * 180/np.pi), -15, 0)

    elif rover_goal(Rover) == 'right' and (len(Rover.nav_angles_right) > (Rover.go_forward / 2)):
        steer = np.clip(np.mean(Rover.nav_angles_right * 180/np.pi), 0, 15)

    else:
        steer = random.randrange(-1, 2, 2) * 15 * 180/np.pi

    return steer

def steer_foward(Rover):
    if rover_goal(Rover) == 'left' and (len(Rover.nav_angles_left) > (Rover.stop_forward / 2)):
        steer = np.clip(np.mean(Rover.nav_angles_left * 180/np.pi), -15, 0) 

    elif rover_goal(Rover) == 'right' and (len(Rover.nav_angles_right) > (Rover.stop_forward / 2)):
        steer = np.clip(np.mean(Rover.nav_angles_right * 180/np.pi), 0, 15)

    else:
        steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

    return steer

def rover_goal(Rover):

    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    
    worldmap_naved = Rover.worldmap[:, :, 2]
    worldmap_mask = np.ones_like(worldmap_naved) - worldmap_naved

    # worldmap -> empty area
    # random select empty area, 
    # then calculate the vector with the origin of current position
    angles = np.zeros((200, 200))
    
    for y in range(0, 200):
        for x in range(0, 200):
            angles[y, x] = np.arctan2(y - ypos, x - xpos)
            if angles[y, x] < 0:
                angles[y, x]+=2*np.pi
    
    relative_angles = angles - yaw
    relative_angles = np.multiply(relative_angles, worldmap_mask)
    
    relative_angles_ = []
    for y in range(0, 200):
        for x in range(0, 200):
            if not relative_angles[y, x] == 0:
                relative_angles_.append(relative_angles[y, x])
            
    relative_angles_avg = np.mean(relative_angles_)
    print(relative_angles_avg)
    
    if relative_angles_avg > 0:
        goal = 'left'

    else:
        goal = 'right'
    
    return goal
    
def predicted_area(Rover):
    prediction_threshold = 200 # maximum is 561 acoording to the analysis in notebook

    dst_size = 5
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw


    if np.sum(Rover.world_naving == Rover.world_naved) >= prediction_threshold:
        prediction = True
    else:
        prediction = False

    return prediction
    

    
    


    
    
