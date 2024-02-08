import map
from map import DIRECTION

DIR = {'North':1,'East':2,'South':3,'West':4}
DIR_INV = {1:'North', 2:'East', 3:'South', 4:'West'}

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

waveform_planning(map,[0,0],[7,7])

