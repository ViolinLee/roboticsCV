## Rover Project
### Obstacle and Rock Identification
These works were first completed on notebook, and some of them were learned from Project Walkthrough Video. The following is the code to identify the rock.    

    def find_rocks(img, levels=(110, 110,50)):
        rockpix = ((img[:,:,0] > levels[0]) & (img[:,:,1] > levels[1]) & (img[:,:,2] < levels[2]))
    
        color_select = np.zeros_like(img[:,:,0])
        color_select[rockpix] = 1
    
        return color_select
    
### Rrocess Image   
The function mainly consists of 7 steps, as shown below.    
1. Define source and destination points for perspective transform    
    
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
2. Apply perspective transform   

        warped, unkown_mask = perspect_transform(img, source, destination)

3. Apply color threshold to identify navigable terrain/obstacles/rock samples   

        threshed = color_thresh(warped) # identify navigable terrain
        obstacle_and_unkown = 1 - np.float32(threshed)
        obs_map = obstacle_and_unkown * unkown_mask

4. Convert thresholded image pixel values to rover-centric coords    

        xpix, ypix = rover_coords(threshed)
        obsxpix, obsypix = rover_coords(obs_map)

5. Convert rover-centric pixel values to world coords    

        world_size = data.worldmap.shape[0]
        scale = 2 * dst_size
        xpos = data.xpos[data.count]
        ypos = data.ypos[data.count]
        yaw = data.yaw[data.count]
    
        x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, xpos, ypos, yaw, world_size, scale)
   
6. Update worldmap (to be displayed on right side of screen)    

        data.worldmap[y_world, x_world, 2] = 255
        data.worldmap[obs_y_world, obs_x_world, 0] = 255
        nav_pix = data.worldmap[:, :, 2] > 0
        data.worldmap[nav_pix, 0] = 0
        rock_map = find_rocks(warped, levels=(110, 110, 50))
        if rock_map.any():
            rock_x, rock_y = rover_coords(rock_map)
            rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
        
            data.worldmap[rock_x_world, rock_y_world, :] = 255   # all channel --> brighter
7. Make a mosaic image, below is some example code    

        output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
        output_image[0:img.shape[0], 0:img.shape[1]] = img
        output_image[0:img.shape[0], img.shape[1]:] = warped
        map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
        output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)
        cv2.putText(output_image,"Populate this image with your analyses to make a video!", (20, 20), 
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)

The output video can be viewed from the Vedio folder. In *perception.py* in project, the above code has made some changes. For example, when updating worldmap, use the way of *+=1* instead of directly assigning 255. But the basic idea is consistent.    
    
![](https://i.imgur.com/qxEAz7H.png)
### Perception   
The code here is very similar to the code in the process_image function above, but there are still several changes or additions that are critical to the mapping result.    
####1. Optimizing Map Fidelity    
The main idea is to save some pictures that can really be used to update worldmap. Although I have thought of some other methods, the following code is simple and effective.    

        if (Rover.pitch <= 360 and Rover.pitch >=359) or (Rover.pitch <= 1 and Rover.pitch >=0):
            if (Rover.roll <= 360 and Rover.roll >=358.5) or (Rover.roll <= 1.5 and Rover.roll >=0):
                Rover.worldmap[y_world, x_world, 2] += 10
                Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    
####2. Update Newly Added Attributes
In addition, there are some code here to update the newly added attributes, which are used in decision step. These new attributes are critical to determining the steering of rover. Especially when avoiding obstacles, what we need more is pixel information nearby, so at some situation, we  use the attribute *self.nav_angles_close* insdead of *self.nav_angles*.     

    if len(angles)>0:
        Rover.nav_angles_close = angles[dist < np.mean(dist)]
    else:
        Rover.nav_angles_close = 0   


### Decision   
####1. Optimizing Time    
I mainly optimize time by proper throttle adjustment, steer manipulation and obstacle avoidance. Throttle adjustment and directional operation are favorable for path optimization. if rover is stuck on an obstacle unfortunately, it may take some time to get rid of it.   

####2. Optimizing % Mapped    
In order to make Rover travel more purposeful, that is, rover will automatically travel to a small area of mapping, I define the *rover_goal* function. This function should play a better role in the *forward* state, but the debugging result is not ideal, it is not used in forward state. However, it is used in the *stop* state, and it is used in conjunction with another equation called *steer_stop*. The *rover_goal* function is tested in notebook *Rover Goal Test.ipynb*.    
    
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

Also, for the decision process, see decition_v2.py, in order to make better decisions, I applied these new additions here - *self.decelerate_forward*, *self.obs_forward*, *self.nav_angles_close*, and *self.k_obs*.   
Among this, I think the way to deal with *self.nav_angles_close*, *self.k_obs* and *self.obs_forward* to avoid the small stone on the road is not bad! Sometimes there are only small obstacles in the middle part of the road, and both sides of the road are very flat. At this time, if we follow the old method, rover will drive through the obstacles in the middle. But use the bellow method, rover will slow down first and then chooses one side to drive. The self.k_obs is used here to amplify the steering angle.    

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.decelerate_forward:
                if np.sum(Rover.nav_angles_close[np.abs(Rover.nav_angles_close* 180/np.pi)<=30])>=Rover.obs_forward:  
                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle 
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = np.clip(np.mean(Rover.nav_angles_close* 180/np.pi), -10, 10)
                    # Set steering to average angle clipped to the range +/- 15
                    #Rover.steer = np.clip(steer_foward(Rover), -15, 15)
                else:
                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle 
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set/2
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    #Rover.steer = steer_stop(Rover)
                    Rover.steer = np.clip(np.mean(Rover.nav_angles_close* 180/np.pi), -15, 15) * Rover.k_obs


### Simulation Result
After determining the final solution and debugging all the parameters, I did five tests，and they all meet rubric. The simulation results are as follows:  
#### Test1
![](https://i.imgur.com/P9FOXnB.jpg)   
#### Test2   
![](https://i.imgur.com/dgXdytG.jpg)
#### Test3    
![](https://i.imgur.com/ABmUaLH.jpg)
#### Test4    
![](https://i.imgur.com/zNMcAKV.jpg)
#### Test5    
![](https://i.imgur.com/GPMuWqY.jpg)    
   
I believe that if there is enough time for me to debug, the result will be better. For example, when the state is' forward ', consider the guiding role of goal. Originally, I intend to add function 'steer_forward' in my code, but the debugging result is not ideal, and it is not used here. If steer_forward can work, it can greatly reduce the randomness of navigation and make it more purposeful. Another way to change the simulation results is to adjust the values of some attributes of Rover, such as the value of Rover.go_forward.


