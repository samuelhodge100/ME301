import map
from map import DIRECTION


def waveform_planning(map,start,end):
    # Start (i,j) in map coordiates
    # End (i,j) in map coordinates

    # Starting at end loop
    path_found_bool = False
    N = 0 
    while(not path_found_bool):
        waveform_check_neighbors(map,[end])
        path_found_bool = True
        pass


def waveform_check_neighbors(map, current_positions):
    # return an array of positions
    
    # from current position check each of the neighbors for an obsticle and if it is already populated
    for position in current_positions:
        # Check each of the cardinal directions
        for dir in range(1,5):
            print(dir)


    return 0 


map = map.EECSMap()
map.printObstacleMap()
map.printCostMap()

waveform_planning(map,[0,0],[8,8])

