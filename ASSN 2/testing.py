import map
from map import DIRECTION
import numpy as np

DIR = {'North':1,'East':2,'South':3,'West':4}
DIR_INV = {1:'North', 2:'East', 3:'South', 4:'West'}

def planning(map, start, end):
    map.setCost(end[0], end[1], 1)
    neighbor_queue = [[[end[0], end[1]], 1]]

    set_costs(neighbor_queue, map, start)
    map.printCostMap()
    #actually tell thing to move
    curr = start
    while (not (curr[0] == end[0] and curr[1] == end[1])):
        dir = 1
        min = 1000
        for x in range(4):
            curr_cost = map.getNeighborCost(curr[0], curr[1], x+1)
            print(curr_cost)
            if (curr_cost != 0 and curr_cost < min):
                dir = x+1
                min = curr_cost
        move(curr[2], dir)
        next_loc = next_loc(curr[0], curr[1], dir)
        curr = [next_loc[0], next_loc[1], dir]
        print(*curr, sep = ", ")
    print(*curr, sep = ", ")
    #check for final dir
    if (not (curr[2] == end[2])):
        turn_count = check_dir(curr[2], end[2])
        turn(turn_count)

    
def set_costs(neighbor_queue, map, start):
    print(*neighbor_queue, sep = ", ")
    curr_element = neighbor_queue.pop(0)
    print(*curr_element, sep = ", ")
    curr = curr_element[0]
    print(*curr, sep = ", ")
    distance_from_end = curr_element[1]
    if (not (curr[0] == start[0] and curr[1] == start[1])):
        distance_from_end += 1
        for x in range (4):
            print(x)
            if(map.getNeighborObstacle(curr[0], curr[1], x+1) == 0):
                if (map.getNeighborCost(curr[0], curr[1], x+1) == 0):
                    map.setNeighborCost(curr[0], curr[1], x+1, distance_from_end)
                    neighbor_queue.append([next_loc(curr[0], curr[1], x+1), distance_from_end])
                    print(f'Cost {distance_from_end} set at position {curr}')
                else: 
                    print(f'There is a cost at position {map_move(curr,x+1)}')
            else:
                print(f'There is an obsticle to the {DIR_INV[x+1]} at position {curr}')
        map.printObstacleMap()
        map.printCostMap()
                    
    if(len(neighbor_queue)>0):
        set_costs(neighbor_queue, map, start)
        # create a structure with al points of grid
        # explore neighboring spots from the end, add 1 to their val
        # continue with next neighboring, add 2....
        # stop when you have gotten to the start
        # now trace back to the goal by choosing lowest value spots
        # yay.
def next_loc(i, j, dir):
    end_i = i
    end_j = j
    if (dir == 1):
        end_i -= 1
    if (dir == 2):
        end_j += 1
    if (dir == 3):
        end_i += 1
    if (dir == 4):
        end_j -= 1
    return [end_i, end_j]

def turn(turn_count):
    if (turn_count > 0):
        for x in range(turn_count):
            print("turning right")
    else:
        print("turning left")

#move one unit in a specific direction based on current orientation
def move(dir_curr, dir_next):
    # north - 1, east - 2, south - dir, west - 4
    print("move")
    if(dir_curr != dir_next):
        turn_count = check_dir(dir_curr, dir_next)
        turn(turn_count)
    print("walk forward")

def check_dir(dir_curr, dir_next):
    # turning right by 90 = 1, left by 90 = -1
    # right by 180 = 2, left by 180 = -2
    test_dir = dir_curr
    print(test_dir)
    turn_count = 0
    while(test_dir != dir_next):
        if (test_dir < 4):
            test_dir += 1
        else:
            test_dir = 1
        print(test_dir)
        turn_count += 1
    if (turn_count > 2):
        turn_count = -1
    return turn_count


def waveform_planning(map,start_position,end_position):
    # Start (i,j) in map coordiates
    # End (i,j) in map coordinates
    # Set initial cost of end_position to be 1
    map.setCost(end_position[0],end_position[1],1)
    # Starting at end loop
    path_found_bool = False
    N = 0 
    val = 2
    while(not path_found_bool):
        waveform_check_neighbors(map,[end_position],val)
        path_found_bool = True
        pass

def map_move(position,dir):
    # Inputs a position and a direction and outputs the updated position
    if dir==1: # North
        position = [position[0]-1, position[1]]
    elif dir==2: # East
        position = [position[0],position[1]+1]
    elif dir==3: # South
        position = [position[0]+1,position[1]]
    elif dir==4: # West
        position = [position[1], position[1]-1]
    return position

def waveform_check_neighbors(map, current_positions,val):
    # return an array of positions

    # from current position check each of the neighbors for an obsticle and if it is already populated
    # Check each of the cardinal directions
    new_positions = []
    for current_position in current_positions:
        for dir in range(1,5):
            if map.getNeighborObstacle(current_position[0],current_position[1],dir):
                print(f'There is an obsticle to the {DIR_INV[dir]} of position {current_position}')
            elif(map.getNeighborCost(current_position[0],current_position[1],dir) != 0):
                print(f'The cell {map_move(current_position,dir)} is already occupied with a cost!')
            else:
                # Assign a cost to that cell and add it to the positions list
                map.setNeighborCost(current_position[0],current_position[1],dir,val)
                new_positions.append(map_move(current_position,dir))
                
                map.printObstacleMap()
                map.printCostMap()
                print('\n')
    print(f'Here are the positions being tracked : {new_positions}')
    val+=1
    waveform_check_neighbors(map,new_positions,val)
                
    
    return 0 

 
map = map.EECSMap()

map.printObstacleMap()
map.printCostMap()

#waveform_planning(map,[0,0],[7,7])
planning(map, [0,0,3], [7,7,1])



def front_and_rear_steering(R,max_wheel_speed):
    # R is the radius of turning horizontally from the center of mass (positive value indicates right turn) in centimeters
    # max_wheel_speed is inputted velocity of the outter wheels as they rotate about the center of rotation TODO: MAKE SURE THE INPUTTED VELOCITIES ARE IN CENTIMETERS PER SECOND
    # Theta defined from vertical axis positive use right hand rule for positive angles

    # Returns a list of [wheel velocities, wheel angles]

    # Define some constants TODO: UPDATE THESE WITH PROPER VALUES
    x0 = 12 # Horizontal distance to center of wheels in centimeters
    y0 = 20 # Vertical distance to the center of wheels in centimeters

    # R = 0 indicates forward or reverse motion

    if R == 0:
        # walk forward or backward using the max_wheel_speed as linear velocity
        pass
        # return [[phi_dot_right,phi_dot_left],[theta_right_front,theta_left_front,theta_right_back,theta_left_back]]
    else:
        # Compute other wheel speed and generic angles:
        phi_undecided = (np.sqrt((abs(R)+x0)**2+y0**2)/np.sqrt((abs(R)-x0)**2)+y0**2)* max_wheel_speed
        theta_undecided_inner = np.arctan(y0/(abs(R)-x0))
        theta_undecided_outter = np.arctan(y0/(abs(R)+x0))
        omega = max_wheel_speed * (abs(R)+x0)
        # Compute odometry for the center of mass

        if R > 0:
            # Right turn
            phi_dot_right = phi_undecided
            phi_dot_left = max_wheel_speed
            theta_right_front = theta_undecided_inner
            theta_left_front = theta_undecided_outter
            theta_right_back = -theta_undecided_inner
            theta_left_back =  -theta_undecided_outter
            return [[phi_dot_right,phi_dot_left],[theta_right_front,theta_left_front,theta_right_back,theta_left_back]]
    
        else:
            # Left Turn
            phi_dot_right = max_wheel_speed
            phi_dot_left = phi_undecided
            theta_right_front = theta_undecided_outter
            theta_left_front = theta_undecided_inner
            theta_right_back = -theta_undecided_outter
            theta_left_back =  -theta_undecided_inner
            return [[phi_dot_right,phi_dot_left],[theta_right_front,theta_left_front,theta_right_back,theta_left_back]]


def odometry(W, R, dt):
    # Inputs:
    # W is the angular velocity of the COM about the turning point
    # R is the radius from the COM to the the turning point
    # dt is the time span in which the control loop functions
    # Need the current state of the robot [x,y,theta] have stored in the class
    # The updates to the current state

    if R==0:
        # Moving straight (based on orientation tho)
        self.state.theta += 0
        self.state.x += np.cos(self.state.theta)*forward_vel*dt
        self.state.y += np.sin(self.state.theta)*forward_vel*dt

        pass
    else:
        # Turning
        self.state.theta += W * dt
        
        # Use the change in and the radius to compute the change in x and y
        self.state.x += np.cos(self.state.theta)*linear_velocity_in_x*dt
        self.state.y += np.sin(self.state.theta)*linear_velocity_in_y*dt

        
def path_control(map):
    # Inputs 
    # map: the obsticle map
    
    # Step one: Given where you are in the map. What sensor values should you be reading
    # Step two: update postion in the world based on sensor vals
    # Step three: Make corrections to robot position

    pass