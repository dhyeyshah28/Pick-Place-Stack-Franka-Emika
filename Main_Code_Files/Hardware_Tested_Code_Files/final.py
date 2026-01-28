import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

#Student imports
from lib.IK_position_null import IK
from lib.calculateFK import FK

##################################################--HELPER FUNCTIONS--##################################################

def calculate_q_via_ik(pos, q_start, verify=True):
    q_end, rollout, success, message = ik.inverse(pos, q_start, method='J_pseudo', alpha = 0.5)
    
    if verify:
        if success:
            return q_end
        else:
            print('Failed to find IK Solution: ')
            print('pos: ', pos)
            print('q_start: ', q_start)
            return None
    
    return q_end


def swap_columns(matrix, col1, col2):
        matrix[:, [col1, col2]] = matrix[:, [col2, col1]]
        return matrix


def rotation_matrix_to_angle_axis(rotation_matrix):
        # Extract the 3x3 rotation matrix
        rotation = rotation_matrix[:3, :3]
        abs_rotation = np.abs(rotation)
    
       # Find the column closest to [0, 0, 1]
        target_column = np.array([0, 0, 1])
        min_diff = 0.2  
        col_to_swap = -1
    
         # Loop through each column to find the one closest to [0, 0, 1]
        for i in range(3):
            # Calculate the absolute Euclidean distance to [0, 0, 1]
            diff = np.linalg.norm(np.abs(abs_rotation[:, i] - target_column))  # Absolute distance to [0, 0, 1]
        
            # If this column is the closest, update the swap index
            if diff < min_diff:
                min_diff = diff
                col_to_swap = i             
    
        # Swap the column that is closest to [0, 0, 1] with the 3rd column
        rotation = swap_columns(rotation, col_to_swap, 2)
    
        # Calculate the rotation angle about the z-axis using atan2
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
        
    
        # Optionally adjust the angle to stay within [-pi, pi]
        if rz >= np.pi / 4:
            rz = rz - np.pi / 2

        elif rz < -np.pi / 4:
            rz = rz + np.pi / 2
        
        return rz

def fixOffset(block_world):

    for i in range(len(block_world)):
        block_world[i][0, 3] -= x_offset
        block_world[i][1, 3] -= y_offset
        block_world[i][2, 3] = z_fixed

    return block_world


def dynamic_adjustment(x,y, w_t, r_threshold = 0.19):
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)

    if r<r_threshold:
        return None, None
    
    # Update the angle
    theta += w_t
    
    # Convert back to Cartesian
    return r * np.cos(theta), r * np.sin(theta)


##################################################--DETECTIONS--##################################################

def comp_filter(curr_reading, prev_reading, alpha):
    filt_pose = (alpha*curr_reading) + ((1 - alpha)*prev_reading)
    return filt_pose

def get_block_world_comp_filter(q):

    alpha=comp_filter_alpha
        
    H_ee_camera = detector.get_H_ee_camera() # Camera in terms of end_effector
    H_c_ee = H_ee_camera 
    _,H_ee_w = fk.forward(q) # End-Effector in terms of world
    H_c_w = H_ee_w @ H_c_ee # Camera in terms of world
    # rospy.sleep(2)
    
    for i in range(5):
            
        b_reading_1 = []
        b_reading_2 = []
        
        # detector.get_detections() = Block in terms of camera
        block_det = detector.get_detections()
            
        for (name_1, pose_1) in block_det:
            b_reading_1.append(pose_1)

        b_reading_1 = np.array(b_reading_1)

        for (name_2, pose_2) in block_det:
            b_reading_2.append(pose_2)

        b_reading_2 = np.array(b_reading_2)

        b_reading_comp = comp_filter(b_reading_2, b_reading_2, alpha)
        b_reading_1 = b_reading_2
        b_reading_2 = b_reading_comp

    block_pose_world=[] # Block in terms of world
    
    for i in range(b_reading_comp.shape[0]):
        block_pose_world.append(H_c_w @ b_reading_comp[i])

        # Returns Block in terms of world
    
    # Consider only the points that are within the workspace
    block_pose_world = [block for block in block_pose_world if block[2,3] > 0.1 and block[2,3] < 0.4]

    if team == 'red':
        # Sort the blocks based on the y position in reverse order
        block_pose_world = sorted(block_pose_world, key=lambda x: x[1,3], reverse=True)

    elif team == 'blue':
        block_pose_world = sorted(block_pose_world, key=lambda x: x[1,3])

    return len(block_pose_world),block_pose_world

def get_block_world(q_current, num_detects=1):

    block_world = {}

    H_ee_camera = detector.get_H_ee_camera()

    for _ in range(num_detects):
        for (name, pose) in detector.get_detections():

            ee_block = H_ee_camera @ pose

            _, T0e = fk.forward(q_current)

            cur_block = T0e @ ee_block

            # see if name key exists in block_world
            if name in block_world:
                block_world[name].append(cur_block)
            else:
                block_world[name] = [cur_block]

    final_block_world = []
    for key in block_world:
        block_worlds = block_world[key]        

        # Average the block world rotation
        block_worlds_R = [mat[:3,:3] for mat in block_worlds]
        avg_R = np.mean(block_worlds_R, axis=0)
        U, S, Vt = np.linalg.svd(avg_R)
        avg_R = U @ Vt
        
        # Average the block world position
        block_worlds_T = [mat[:3,3] for mat in block_worlds]
        avg_T = np.mean(block_worlds_T, axis=0)
        
        avg_block_world = np.eye(4)
        avg_block_world[:3,:3] = avg_R
        avg_block_world[:3,3] = avg_T

        final_block_world.append(avg_block_world)

    if team == 'red':
        # Sort the blocks based on the y position in reverse order
        final_block_world = sorted(final_block_world, key=lambda x: x[1,3], reverse=True)

    elif team == 'blue':
        final_block_world = sorted(final_block_world, key=lambda x: x[1,3])
    
    final_block_world = [block for block in final_block_world if block[2,3] > 0.1 and block[2,3] < 0.4]

    block_count = len(final_block_world)

    return block_count, final_block_world

##################################################--ROBOT MOVEMENTS--##################################################

def drop_block(distance=0.09, force=10):
    arm.exec_gripper_cmd(distance, force)

def grab_block(distance=0.048, force=52):
    arm.exec_gripper_cmd(distance, force)

def move_to_place(T, q_current):
        if team == 'red':
            place_location = np.array(([1,0,0, 0.562],
                        [0,-1,0, 0.2], 
                        [0,0,-1,0.23 + T*0.053],
                        [0,0,0,1]))
            
            if T >= 5:
                place_location = np.array(([1,0,0, 0.562],
                        [0,-1,0, 0.2], 
                        [0,0,-1,0.23 + T*0.054],
                        [0,0,0,1]))
        else:
            place_location = np.array(([1,0,0, 0.562],
                        [0,-1,0, -0.2], 
                        [0,0,-1,0.23 + T*0.053],
                        [0,0,0,1]))
            if T >= 5:
                place_location = np.array(([1,0,0, 0.562],
                        [0,-1,0, -0.2], 
                        [0,0,-1,0.23 + T*0.054],
                        [0,0,0,1]))

        q_place = calculate_q_via_ik(place_location, q_current)

        return q_place

##################################################--STATIC--##################################################

def move_to_static_block(block, q_current, z_above_block = 0.4, z_block=0.225):

        ee_rot = np.array(([1,0,0],
	    			[0,-1,0], 
	    			[0,0,-1],
	    			[0,0,0]))
        block_pos = block[:,3]
        block_pos = block_pos.reshape(4,1)
        ee_goal = np.hstack((ee_rot,block_pos))

        angle = rotation_matrix_to_angle_axis(block[:3,:3])       

        ee_align = deepcopy(ee_goal)
        ee_align[2, 3] = z_above_block
        
        q_align = calculate_q_via_ik(ee_align, q_current)
        if q_align is not None:
            q_align[-1] = q_align[-1] - angle
        else:
            return None, None
        
        ee_goal[2, 3] = z_block

        q_block = calculate_q_via_ik(ee_goal, q_align)
        if q_block is not None:
            q_block[-1] = q_block[-1] - angle 
        else:
            return None, None
        
        return q_align, q_block

def pick_place_static(q_above_pickup, q_drop, stack_block_num=4):

    # Move to the above pickup position
    print("Moving to above pickup position")
    arm.safe_move_to_position(q_above_pickup)

    # Get the block world position
    print("Getting the block world position")
    if use_comp_filter:
        block_count, block_world = get_block_world_comp_filter(q_above_pickup)
    else:
        block_count, block_world = get_block_world(q_above_pickup, num_detects=5)

    block_world = fixOffset(block_world)

    # Open the gripper
    print("Opening the gripper")
    drop_block(drop_ee_dist, drop_ee_force)

    iteration = 0
    
    while block_count > 0:
        
        ####################################################################################################

        # Pick Sequence
        print("Starting the pick sequence")

        # Move to the block
        print("Moving to the block")

        q_align, q_block = move_to_static_block(block_world[0], q_above_pickup)

        if q_align is None or q_block is None:
            print("Failed to find IK Solution")

            # remove the block from the block world
            block_world.pop(0)
            block_count -= 1
            continue

        arm.safe_move_to_position(q_align)
        arm.safe_move_to_position(q_block)

        # Close the gripper
        print("Closing the gripper")
        grab_block(grab_ee_dist, grab_ee_force)

        #################################################################################################### 

        # Place Sequence
        print("Starting the place sequence")

        # Move to the above drop position
        print("Moving to above drop position")
        arm.safe_move_to_position(q_drop)

        # # Detect where to place from the block world
        # print("Detecting where to place")
        # target_block_count, target_block_world = get_block_world(q_drop, num_detects=2)

        # if target_block_count == 0:
        #     q_place = move_to_place(0, q_drop)
        #     z_value = 0
        #     print("z_value: ", z_value)
        # else:
        #     z_value =  round((max([block[2,3] - red_black_box_height for block in target_block_world]) * scaling_factor))            
        #     print("z_value: ", z_value)
        #     q_place = move_to_place(z_value, q_drop)


        # Detect where to place from the iteration
        q_place = move_to_place(iteration, q_drop)
        
        if q_place is None:
            print("Failed to find IK Solution for q_place")
            # USE EMERGENCY Q
            q_place = emergency_qs[f'q_place_{iteration+1}']

        print("\n\n")
        print("ADD TO EMERGENCY IF WORKED IN TESTING PHASE")
        print("Iteration: ", iteration)
        print("q_place: ", q_place)
        print("\n\n")

        # Move to the place location
        print("Moving to the place location")
        arm.safe_move_to_position(q_place)

        # Drop the block
        print("Dropping the block")
        drop_block(drop_ee_dist, drop_ee_force)

        ####################################################################################################
        
        # Reset sequence 
        arm.safe_move_to_position(q_drop)

        iteration += 1

        if iteration == stack_block_num:
            break

        arm.safe_move_to_position(q_above_pickup)

        if use_comp_filter:
            block_count, block_world = get_block_world_comp_filter(q_above_pickup)
        else:
            block_count, block_world = get_block_world(q_above_pickup, num_detects=5)
    
        block_world = fixOffset(block_world)

##################################################--DYNAMIC--##################################################

def move_to_dynamic_block(block, q_current):

    ee_rot = np.array(([1,0,0],
        			[0,-1,0], 
        			[0,0,-1],
                    [0, 0, 0]))
    block_pos = block[:,3]
    block_pos = block_pos.reshape(4,1)
    ee_goal = np.hstack((ee_rot,block_pos))

    x = ee_goal[0, 3]
    y = ee_goal[1, 3]
    if team == 'red':
        y -= y_adjustment
    else:
        y += y_adjustment

    xn,yn = dynamic_adjustment(x,y, w_t_value, r_threshold_value)

    if xn is None:
        return None
    
    if team == 'red':
        yn += y_adjustment
    else:
        yn -= y_adjustment

    ee_goal[0, 3] = xn
    ee_goal[1, 3] = yn
    ee_goal[2, 3] = zn_fixed

    q_block = calculate_q_via_ik(ee_goal, q_current)
    if q_block is not None:
        if team == "red":    
            if q_block[-1] - pi < 2.897 and q_block[-1] - pi > -2.897:
                q_block[-1] = q_block[-1] - pi
            else:
                q_block[-1] = q_block[-1] + pi

    return q_block

def pick_place_dynamic(q_above_rotate, q_above_drop_stacked, stack_block_num=6):

    # Move to the above rotate position
    print("Moving to above rotate position")
    arm.safe_move_to_position(q_above_rotate)

    # Open the gripper
    print("Opening the gripper")
    arm.open_gripper()

    # Get the block world position
    print("Getting the block world position")
    if use_comp_filter:
        block_count, block_world = get_block_world_comp_filter(q_above_rotate)
    else:
        block_count, block_world = get_block_world(q_above_rotate, num_detects=5)

    block_world = fixOffset(block_world)

    blocks_detected_at = time_in_seconds()

    movement_made_at = time_in_seconds()

    # Asuming all 4 static blocks are stacked
    iteration = 4

    while True:

        # if (block_count == 0 and time_in_seconds() - blocks_detected_at > 20) or (time_in_seconds() - movement_made_at > 60):
        #     # Use emergency q and try luck method ... we can also add if no movement is made in 60 seconds then same condition
        #     break

        if block_count == 0:
            rospy.sleep(1)
            if use_comp_filter:
                block_count, block_world = get_block_world_comp_filter(q_above_rotate)
            else:
                block_count, block_world = get_block_world(q_above_rotate, num_detects=5)
            block_world = fixOffset(block_world)
            continue
        
        else:
            blocks_detected_at = time_in_seconds()
            print("Block Detected: ", block_count)
        ####################################################################################################

        # Pick Sequence
        print("Starting the pick sequence")

        # Move to the block
        print("Moving to the block")
        q_block = move_to_dynamic_block(block_world[-1], q_above_rotate)

        if q_block is None:
            rospy.sleep(1)
            if use_comp_filter:
                block_count, block_world = get_block_world_comp_filter(q_above_rotate)
            else:
                block_count, block_world = get_block_world(q_above_rotate, num_detects=5)
            block_world = fixOffset(block_world)
            continue
        
        arm.safe_move_to_position(q_block)

        movement_made_at = time_in_seconds()

        # Close the gripper
        print("Closing the gripper")
        grab_block(grab_ee_dist, grab_ee_force)

        ####################################################################################################

        # Place Sequence
        print("Starting the place sequence")
        arm.safe_move_to_position(q_above_drop_stacked)

        # Detect where to place from the block world
        print("Detecting where to place")
        if use_comp_filter:
            target_block_count, target_block_world = get_block_world_comp_filter(q_above_drop_stacked)
        else:
            target_block_count, target_block_world = get_block_world(q_above_drop_stacked, num_detects=2)

        target_block_world = fixOffset(target_block_world)

        # if target_block_count:

        #     z_value =  round((max([block[2,3] for block in target_block_world]) * scaling_factor))            
        #     print("z_value: ", z_value)
        #     q_place = move_to_place(z_value, q_above_drop)
        #     # if z_value >= 4:
        #     #     q_place = move_to_place(z_value, q_above_drop)
        #     # else:
        #     #     q_place = move_to_place(iteration, q_above_drop)
        #     # if q_place is None:
        #     #     print("Failed to find IK Solution for q_place")
        #     #     # USE EMERGENCY Q
        #     #     q_place = emergency_qs[f'q_place_{iteration+1}']
        
        # else:
        
        q_place = move_to_place(iteration, q_above_drop)

        print("\n\n")
        print("ADD TO EMERGENCY IF WORKED IN TESTING PHASE")
        print("Iteration: ", iteration)
        print("q_place: ", q_place)
        print("\n\n")

        # Move to the place location
        print("Moving to the place location")
        arm.safe_move_to_position(q_place)

        # Drop the block
        print("Dropping the block")
        arm.open_gripper()

        ####################################################################################################

        # Reset sequence
        print("Resetting the sequence")
        arm.safe_move_to_position(q_above_drop_stacked)

        iteration += 1

        if iteration == stack_block_num:
            break

        arm.safe_move_to_position(q_above_rotate)

        if use_comp_filter:
            block_count, block_world = get_block_world_comp_filter(q_above_rotate)
        else:
            block_count, block_world = get_block_world(q_above_rotate, num_detects=5)


##################################################--SETUP FUNCTIONS--##################################################

def set_static_view(q_current):

    if team == 'red':
        pos_above_pickup = np.array(([1, 0, 0, 0.52 ],
                                        [0,-1, 0, -0.2 ], 
                                        [0, 0,-1, 0.52 ], 
                                        [0, 0, 0, 1    ]))

        pos_above_drop = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0, 0.2  ], 
                                    [0, 0,-1, 0.45  ],
                                    [0, 0, 0, 1    ]))
    
    else:
        pos_above_pickup = np.array(([1, 0, 0, 0.52 ],
                                        [0,-1, 0, 0.2  ], 
                                        [0, 0,-1, 0.52 ],
                                        [0, 0, 0, 1    ]))
        
        pos_above_drop = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0,-0.2  ], 
                                    [0, 0,-1, 0.45  ],
                                    [0, 0, 0, 1    ]))

    q_above_pickup = calculate_q_via_ik(pos_above_pickup, q_current)

    if (q_above_pickup == q_current).all():
        q_above_pickup = calculate_q_via_ik(pos_above_pickup, q_current)


    q_above_drop = calculate_q_via_ik(pos_above_drop, q_current)

    if (q_above_drop == q_current).all():
        q_above_drop = calculate_q_via_ik(pos_above_pickup, q_current)

    return q_above_pickup, q_above_drop


def set_dynamic_block_view(q_current):

    if team == 'red':
        pos_above_rotate = np.array(([1, 0, 0, 0   ],
                                    [0,-1, 0, 0.7 ],
                                    [0, 0,-1, 0.4 ],
                                    [0, 0, 0, 1   ]))
        
        pos_above_drop_stacked = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0, 0.2  ], 
                                    [0, 0,-1, 0.65  ],
                                    [0, 0, 0, 1    ]))
    
    else:
        pos_above_rotate = np.array(([1, 0, 0, 0   ],
                                    [0,-1, 0, -0.7],
                                    [0, 0,-1, 0.4 ],
                                    [0, 0, 0, 1   ]))
        
        pos_above_drop_stacked = np.array(([1, 0, 0, 0.52 ],
                                    [0,-1, 0,-0.2  ], 
                                    [0, 0,-1, 0.65  ],
                                    [0, 0, 0, 1    ]))
    
    q_above_rotate = calculate_q_via_ik(pos_above_rotate, q_current, verify=True)
    if (q_above_rotate == q_current).all():
        q_above_rotate = calculate_q_via_ik(pos_above_rotate, q_current)
        
    if team == "red":
        if q_above_rotate[-1] - pi < 2.897 and q_above_rotate[-1] - pi > -2.897:
            q_above_rotate[-1] = q_above_rotate[-1] - pi
        else:
            q_above_rotate[-1] = q_above_rotate[-1] + pi

    q_above_drop_stacked = calculate_q_via_ik(pos_above_drop_stacked, q_current)
    if (q_above_drop_stacked == q_current).all():
        q_above_drop_stacked = calculate_q_via_ik(q_above_drop_stacked, q_current)

    return q_above_rotate, q_above_drop_stacked

##################################################--MAIN--##################################################

if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")

    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    if team == 'red':
        emergency_qs = {
            'q_above_pickup': np.array([-1.83376634e-01, 7.90806995e-03, -1.84837988e-01, -1.56523269e+00, 1.45339161e-03,  1.57300605e+00, 4.17185979e-01]),
            'q_above_drop': np.array([0.17320456, -0.01933834, 0.19194961, -1.77723386, 0.00375479, 1.75824931, 1.14981758]),
            'q_above_rotate': np.array([-0.82870465, -0.93406695, 1.72730474, -1.14483025, 0.92926407, 1.43886509, -1.18035082]),
            'q_above_drop_stacked': np.array([0.32585352, 0.25697452, 0.05841297, -0.86094054, -0.01650469, 1.1175494, 1.160523]),

            'q_dynamic_pickup': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),  

            'q_place_1': np.array([ 0.30539492, 0.25206026, 0.03827549, -2.00221296, -0.01230638, 2.25405925, 1.1356298 ]),
            'q_place_2': np.array([ 0.26846675, 0.18736229, 0.07729645, -1.94692244, -0.01700818, 2.13367295, 1.13889039]),
            'q_place_3': np.array([ 0.24293919, 0.14088693, 0.10418676, -1.86919397, -0.01613017, 2.00927691, 1.13834794]),
            'q_place_4': np.array([ 0.22870994, 0.11398602, 0.11935287, -1.76777076, -0.01422182, 1.88092342, 1.13703419]),
            'q_place_5': np.array([ 0.22608491, 0.10852209, 0.12330291, -1.63975945, -0.01353202, 1.74744816, 1.13644607]),
            'q_place_6': np.array([ 0.23668411, 0.12781881, 0.11520717, -1.47876482, -0.01466259, 1.60574168, 1.13687013]),
            'q_place_7': np.array([ 0.26356994, 0.17972047, 0.09176603, -1.26934866, -0.01650491, 1.44834554, 1.13724803]),
            'q_place_8': np.array([ 0.312117  , 0.29223733, 0.04178058, -0.95864048, -0.01267773, 1.25066104, 1.13353644])
        }
    else:
        emergency_qs = {
            'q_above_pickup': np.array([ 1.87884314e-01, 7.90148892e-03, 1.80304215e-01, -1.56523270e+00, -1.41695423e-03, 1.57300610e+00, 1.15358432e+00]),
            'q_above_drop': np.array([-0.18474463, -0.01929637, -0.18053245, -1.77723404, -0.00352629, 1.75825005, 0.42081113]),
            'q_above_rotate': np.array([ 0.84098714, -0.87723034, -1.81075025, -1.1518686, -0.84836095, 1.47756159, -0.41517303]),
            'q_above_drop_stacked': np.array([-0.24049283, 0.25966636, -0.17955265, -0.86067122, 0.05104353, 1.11685786, 0.3936485]),
            
            'q_dynamic_pickup': np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]),

            'q_place_1': np.array([-0.14987351, 0.25686592, -0.20150012, -2.00182408, 0.06553576, 2.25266031, 0.39912742]),
            'q_place_2': np.array([-0.15634514, 0.19028437, -0.19543331, -1.94677001, 0.04342684, 2.13309093, 0.41389785]),
            'q_place_3': np.array([-0.16164282, 0.14260095, -0.18986081, -1.8691338,  0.02962428, 2.00903732, 0.42320238]),
            'q_place_4': np.array([-0.16548465, 0.11510468, -0.18608828, -1.76774132, 0.0223147,  1.88080349, 0.4282202 ]),
            'q_place_5': np.array([-0.16842188, 0.10950131, -0.18476651, -1.63973581, 0.02039553, 1.74735183, 0.42970885]),
            'q_place_6': np.array([-0.1730075 , 0.1290606 , -0.1850433 , -1.47872916, 0.02369644, 1.60560047, 0.42802714]),
            'q_place_7': np.array([-0.18427348, 0.18172611, -0.18489216, -1.26926085, 0.03348328, 1.44802386, 0.42331185]),
            'q_place_8': np.array([-0.21997884, 0.29517078, -0.17147136, -0.95836987, 0.05233214, 1.24986425, 0.41775002])
        }

    start_time = time_in_seconds()

    # Variables
    team = team
    ik = IK()
    fk = FK()

    red_blue_black_box_height = 0.2

    scaling_factor = 20

    drop_ee_dist = 0.09
    drop_ee_force = 10
    
    grab_ee_dist = 0.046
    grab_ee_force = 55

    use_comp_filter = True
    comp_filter_alpha = 0.78

    if team == 'red':
        x_offset = 0
        y_offset = 0
        z_fixed = 0.225
    else:
        x_offset = 0.015
        y_offset = 0
        z_fixed = 0.225
    
    w_t_value = 0.24
    r_threshold_value = 0.19

    y_adjustment = 0.990
    zn_fixed = 0.16

    tune_for_q = False

    # Static Pick and Place
    q_above_pickup, q_above_drop = set_static_view(start_position)

    if q_above_pickup is None:
        print("Failed to find IK Solution for q_above_pickup, using emergency q")
        q_above_pickup = emergency_qs['q_above_pickup']
    
    if q_above_drop is None:
        print("Failed to find IK Solution for q_above_drop, using emergency q")
        q_above_drop = emergency_qs['q_above_drop']

    # Dynamic Pick and Place
    q_above_rotate, q_above_drop_stacked = set_dynamic_block_view(start_position)

    if q_above_rotate is None:
        print("Failed to find IK Solution for q_above_rotate, using emergency q")
        q_above_rotate = emergency_qs['q_above_rotate']
    
    if q_above_drop_stacked is None:
        print("Failed to find IK Solution for q_above_drop_stacked, using emergency q")
        q_above_drop_stacked = emergency_qs['q_above_drop_stacked']

    print("\n")
    print("\n")
    print("ADD TO EMERGENCY IF WORKED IN TESTING PHASE")
    print("q_above_pickup: ", q_above_pickup)
    print("q_above_drop: ", q_above_drop)
    print("q_above_rotate: ", q_above_rotate)
    print("q_above_drop_stacked: ", q_above_drop_stacked)
    print("\n")
    print("\n")

    if tune_for_q:
        arm.open_gripper()
        lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
        upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

        # q_test = deepcopy(q_above_rotate)
        q_test = [-0.96727773, -1.18120756, 1.90118979, -1.10456854, 1.07419833, 1.6682946, -1.19594977]
        arm.safe_move_to_position(q_test)

    while tune_for_q:
        print("Tuning for q_test")
        print("Current q_test", q_test)
        joint_number = input("Enter joint number to tune: ")
        print("Current joint value: ", q_test[int(joint_number)], "   Limits: ", lowerLim[int(joint_number)], " - ", upperLim[int(joint_number)])

        joint_value = input("Enter joint value: ")
        q_test[int(joint_number)] = float(joint_value)

        arm.safe_move_to_position(q_test)

        if input("Continue tuning? (y/n): ") == 'n':
            break  

    if tune_for_q:
        print("Final q_test: ", q_test)

    ####################################################################################################
    static_start_time = time_in_seconds()
    pick_place_static(q_above_pickup, q_above_drop_stacked, stack_block_num=4)
    static_end_time = time_in_seconds()

    print("Time taken for static pick and place: ", static_end_time - static_start_time, " seconds")
    ####################################################################################################

    dynamic_strat_time = time_in_seconds()
    pick_place_dynamic(q_above_rotate, q_above_drop_stacked, stack_block_num=6)
    dynamic_end_time = time_in_seconds()

    print("Time taken for dynamic pick and place: ", dynamic_end_time - dynamic_strat_time, " seconds")
    ####################################################################################################

    end_time = time_in_seconds()
    print(f"Run Time: {end_time - start_time:.2f} seconds")

    # END STUDENT CODE