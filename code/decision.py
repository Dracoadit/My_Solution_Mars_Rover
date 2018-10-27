import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # determin indices where angles are inside -+ 10 degree corridor infront of rover
    cut_lim = 10
    idx = (Rover.nav_angles < cut_lim*np.pi/180) & (Rover.nav_angles >-cut_lim*np.pi/180)
    # determin indices where distances are above 5m
    idx2 = (Rover.nav_dists >= 50)
    # define range when to stop infront of wall
    stop_range = 44 # consider offset of rover to image
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode stati forward, stop, stuck
        if Rover.mode == 'forward':
            
            # Check if rover position is changing. -> Lots of Nonetype errors or inaccurate            
            #R_pos_x = np.round(np.float(Rover.pos[0]),decimals=1)
            #R_pos_y = np.round(np.float(Rover.pos[1]),decimals=1)
            #R_pos_x_last = np.round(np.float(Rover.last_pos[0]),decimals=1)
            #R_pos_y_last = np.round(np.float(Rover.last_pos[1]),decimals=1)
            #if (np.absolute(R_pos_x - R_pos_x_last) <= 0.1) and (np.absolute(R_pos_y - R_pos_y_last) <= 0.1) :
            
            # Check if rover is not moving even though rover tries to accelerate
            if Rover.vel == 0 and Rover.throttle == 0.3:
               Rover.stuck_count +=1
            
            # Check if rover is is moving
            if Rover.stuck_count <=20:
                # Check the extent of navigable terrain and if there is terrain that goes further than the stop_stop range.
                if len(Rover.nav_angles) >= Rover.stop_forward and np.any(Rover.nav_dists>=stop_range):  
                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    # catch incase idx2 results in an empty array
                    try:
                        # Check if maximum angle for distances that are further than 5m is closer than 5 degree. Assumption: if statement is true, the wall on the left is close.
                        if np.max(Rover.nav_angles[idx2]) < 5* np.pi/180: # or Rover.nav_dists[np.max(Rover.nav_angles == np.max(Rover.nav_angles))]<40:
                         # Normal steering if wall is close
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        else:
                         # If wall is not close then steer with 20% of the angles standard deviation to the left.
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)+ 0.22*np.std(Rover.nav_angles*180/np.pi), 0, 15) #clip to allow only turning to the left
                    except ValueError:
                        # When empty array assume rover is stuck.
                        Rover.mode = 'stuck'
                        
                    # Check rover position every 15th iteration
                    #Rover.last_count += 1
                    #if Rover.last_count >=15:
                    #   Rover.last_pos = Rover.pos
                    #    Rover.last_count = 0
                    
                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif len(Rover.nav_angles) < Rover.stop_forward or np.any(Rover.nav_dists<stop_range):
                        # Set mode to "stop" and hit the brakes!
                        Rover.throttle = 0
                        # Set brake to stored brake value
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        Rover.mode = 'stop'
            else:
                # if rover is not moving after 20 iterations, then stop and go to stuck mode.
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stuck'
                # reset count for stuck mode
                Rover.stuck_angle = 0

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward that is further than the stop range.
                if len(Rover.nav_angles) < Rover.go_forward or np.all(Rover.nav_dists<stop_range):
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                # Additionally check if inside the cut_lim range (+-10 degrees) in front of the rover the number of distances that are longer than 5m are above the threshold for going forward.
                if len(Rover.nav_angles) >= Rover.go_forward and len(Rover.nav_dists[idx]>=stop_range)>=Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
                    
        # If we are in stuck mode then do  
        elif Rover.mode == 'stuck':
            # start counting calls of suck mode
            Rover.stuck_angle+=1
            # turn to the right as long as stuck calls are below 12
            if Rover.stuck_angle<=12:
                Rover.steer =-15
                Rover.brake = 0
                Rover.throttle = 0
            # if calls are above 12 then go to forward mode. (if this is not possible, rover will jump back to stuck and continue turning.)
            else:
                Rover.mode = 'forward'
                Rover.stuck_count = 0
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

