import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only

def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_thresh = np.array([0,0,0])
    upper_thresh = np.array([113,100,222])
    collorselect = cv2.inRange(hsv,lower_thresh,upper_thresh)
    #above_thresh = (img[:,:,0] > rgb_thresh[0]) \
    #            & (img[:,:,1] > rgb_thresh[1]) \
    #            & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    #color_select[above_thresh] = 1
    # Return the binary image
    return color_select

#identify terrain:
# color ranges:
# lower_rgb = np.array([[120,100,160],[60,100,140],[0,100,130]])
# upper_rgb = np.array([[120,255,255],[60,255,255],[0,255,255]])

def terrain_ident(img, lower_color, upper_collor):
    #choose red channel
    red_channel = np.copy(img)
    red_channel[:,:,[1, 2]] = 0
    #convert to hsv
    hsv_red = cv2.cvtColor(red_channel, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get defined color range
    terrain_select_red = cv2.inRange(hsv_red, lower_color[0][:],upper_collor[0][:])
    
    # choose green channel
    green_channel = np.copy(img)
    green_channel[:,:,[0, 2]] = 0
    # convert to hsv
    hsv_green = cv2.cvtColor(green_channel, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get defined color range
    terrain_select_green = cv2.inRange(hsv_green, lower_color[1][:],upper_collor[1][:])
    
    
    # choose blue channel
    blue_channel = np.copy(img)
    blue_channel[:,:,[0, 1]] = 0
    # convert to hsv
    hsv_blue = cv2.cvtColor(blue_channel, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get defined color range
    terrain_select_blue = cv2.inRange(hsv_blue, lower_color[2][:],upper_collor[2][:])
    
    # Merge different colors
    idx_terrain = (terrain_select_red == 255) & (terrain_select_green == 255) & (terrain_select_green == 255)
    #env_select= cv2.addWeighted(env_select_green, 1, env_select_red, 1, 0)
    terrain_select = np.zeros_like(img[:,:,0])
    terrain_select[idx_terrain] = 1      
    return terrain_select

# Identify rocks
def rock_ident(img, lower_color, upper_color):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get defined color range
    rock_select = cv2.inRange(hsv,lower_color,upper_color)
    return rock_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel



# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    # Apply transform of array of ones to identify visible area
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1] , img.shape[0]))# keep same size as input image
    return warped, mask

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[15, 140], [302 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples 
    # Define upper and lower bound for terrain
    lower_rgb = np.array([[120,100,160],[60,100,150],[0,100,130]])
    upper_rgb = np.array([[120,255,255],[60,255,255],[0,255,255]])
    # identify terrain
    terrain = terrain_ident(warped, lower_rgb, upper_rgb)
    # identify obstacles
    obstacles = np.absolute(np.float32(terrain)-1) * mask
    # Define upper and lower bound for rock identification
    lower_yellow = np.array([80,90, 90])
    upper_yellow = np.array([100,255,255])
    # identify rock
    rock = rock_ident(warped, lower_yellow, upper_yellow)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,2] = terrain * 255
    Rover.vision_image[:,:,0] = obstacles * 255
    # Rock vision is defined later on in if statement
        

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(terrain)
    
    # Use for mapping only terrain pixels that are inside 5m range
    # Transform into polar coords
    dist, angles = to_polar_coords(xpix, ypix)
    # Find indices to cut distances
    idx_mapping_cut = dist >= Rover.mapping_distance
    #Coppy pixels and set pixels that are further than 5m to zero
    #xpix_cut = np.copy(xpix)
    #ypix_cut = np.copy(ypix)
    #xpix_cut[idx_mapping_cut] = 0
    #ypix_cut[idx_mapping_cut] = 0
    
    # Convert obstacles
    xpix_obs, ypix_obs = rover_coords(obstacles)
    # Cut mapping range for obstacles
    #dist_obs, angles_obs = to_polar_coords(xpix_obs, ypix_obs)
    #idx_obs = dist_obs >= 40
    #xpix_obs_cut = np.copy(xpix_obs)
    #xpiy_obs_cut = np.copy(ypix_obs)
    #xpix_obs_cut[idx_obs] = 0
    #ypix_obs_cut[idx_obs] = 0
    
    #Convert Rocks
    xpix_rock, ypix_rock = rover_coords(rock)
    
    # 6) Convert rover-centric pixel values to world coordinates
    # Get properties
    xpos_rover = Rover.pos[0]
    ypos_rover = Rover.pos[1]
    yaw = Rover.yaw
    scale = 2 * dst_size
    world_size = Rover.worldmap.shape[0]
    # Mapping
    terrain_x_world, terrain_y_world = pix_to_world(xpix, ypix, xpos_rover, ypos_rover,yaw, world_size, scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(xpix_obs, ypix_obs, xpos_rover, ypos_rover,yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(xpix_rock, ypix_rock, xpos_rover, ypos_rover,yaw, world_size, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    navigable_x_world = terrain_x_world 
    navigable_y_world = terrain_y_world
    
    # Set limits for enabling the mapping 
    pitch_lim = 0.6
    roll_lim = 1.2
    vel_lim = 0.2
    # Only map if pitch and roll are smaller than its limit and only if velocity is faster than 0.2m/s or close to zero
    if (Rover.pitch <= pitch_lim or Rover.pitch >= (360 - pitch_lim)) and (Rover.roll <= roll_lim or Rover.roll >= (360 - roll_lim)) and (Rover.vel >= vel_lim or (Rover.vel <= 0.05 and Rover.vel >= 0)):
        # Set pixels that are further than cut distance to zero
        navigable_x_world[idx_mapping_cut] = 0
        navigable_y_world[idx_mapping_cut] = 0
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] +=10
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] +=1
        navigable_world = Rover.worldmap[:,:,2] > 0
        Rover.worldmap[navigable_world,0] = 0
        
        # Check if there are any rocks
        if rock.any():
            # Find center point of rock to remove drift
            rock_dist, rock_ang = to_polar_coords(xpix_rock,ypix_rock)
            rock_idx = np.argmin(rock_dist)
            rock_x_cen = rock_x_world[rock_idx]
            rock_y_cen = rock_y_world[rock_idx]
            Rover.worldmap[rock_y_cen, rock_x_cen, :] = 255
            Rover.vision_image[:,:,1] = rock * 255
        else:
            Rover.vision_image[:,:1] = 0
            
            
            
    # 8) Convert rover-centric pixel positions to polar coordinates
       
    dists, angles = to_polar_coords(xpix, ypix)
    Rover.nav_dists = dists
    Rover.nav_angles = angles
        # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
 
    
    
    return Rover