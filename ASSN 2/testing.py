import map
from map import DIRECTION
import numpy as np



class controller():
    # 2 and 6 front right leg
    # 1 and 5 front left leg
    # 4 and 8 back right leg
    # 3 and 7 back left leg
    # for motors 1,2,3,4 positive theta input is forward direction
    # for motors 5,6,7,8 posiive theta input is downward
    # for legs, standing is 100 deg, lifted is 80, splayed is 60, centered is 0
    def __init__(self,Kp=1,Kd=0,Ki=0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.I = 0
        self.dms_port = 6
        self.dt = 1/10000 # Loop enforced at 10 kHz may need to actually measure as enforement only occurs when loop is faster.
        self.sleep_time = 0.1
        # Position of the robot in real_world coordinates
        self.theta = 0
        self.x = 0
        self.y = 0
        # Position of the robot in map coordinates
        self.i = 0 # Should be values [0,7]
        self.j = 0 # Should be values [0,7]
        self.dir = 0 # Should be values [1,4]
        
        # Intitial Motor Config
        '''
        # Set Motor Modes
        setMotorMode(5,1)
        setMotorMode(6,1)
        setMotorMode(7,1)
        setMotorMode(8,1)

        # Set forward angle
        response = setMotorTargetPositionCommand(1,512)
        response = setMotorTargetPositionCommand(2,512)
        response = setMotorTargetPositionCommand(3,512)
        response = setMotorTargetPositionCommand(4,512)

        # Set motor speeds to zero
        response = setMotorWheelSpeed(5,0)
        response = setMotorWheelSpeed(6,0)
        response = setMotorWheelSpeed(7,0)
        response = setMotorWheelSpeed(8,0)

        rospy.sleep(0.5)
        '''

    def wall_follow(self, threshold = 40, DMS_SENSOR_REF = 1000):
        # General PID Controller Scheme
        # Inputs the sensor_port
        # Wall is to the left

        dms_value = self.scan_left(100)
        error = DMS_SENSOR_REF-dms_value
        rospy.loginfo("The error is: %d", error)
        # Error is negative when the robot is closer to the wall

        if error < 0 - threshold:
            # Turn right
            self.turn_right(self.Kp*20)
            rospy.loginfo("Robot turning right")

        if error > 0 + threshold:
            # Turn left
            self.turn_left(self.Kp*20)
            rospy.loginfo("Robot turning left")

        self.walk()

    def reactive_control(self,DMS_SENSOR_REF = 1700):
        left_value,front_value,right_value = self.full_scan()
        rospy.loginfo("left value %f, front value %f, right value %f", left_value, front_value, right_value)
        # If all are greater than thresh
        if(left_value > DMS_SENSOR_REF and front_value > DMS_SENSOR_REF and right_value > DMS_SENSOR_REF):
            rospy.loginfo("Robot turning around")
            self.turn_180()
        # If right and front blocked
        elif(front_value > DMS_SENSOR_REF and right_value > DMS_SENSOR_REF):
            rospy.loginfo("Robot turning left")
            self.turn_90_left()
        # The other two
        elif(front_value > DMS_SENSOR_REF and left_value > DMS_SENSOR_REF):
            rospy.loginfo("Robot turning right")
            self.turn_90_right()

    def full_scan(self):
        left_value = self.scan_left(100)
        rospy.sleep(self.sleep_time)
        front_value = self.scan_front()
        rospy.sleep(self.sleep_time)
        right_value = self.scan_right(100)
        rospy.sleep(self.sleep_time)
        response = setMotorTargetPositionCommand(10, theta_to_encoder(0,10))
        rospy.sleep(self.sleep_time * 2)
        return left_value,front_value,right_value
    
    def scan_left(self,deg):
        # Assuming that once the motor achieves the commanded motor position it will return true
        # TODO: Need to check this assumption, and integrate into the other scan functions
        response = 0
        while(not response):
            response = setMotorTargetPositionCommand(10, theta_to_encoder(deg,10))
        dms_sensor_reading = getSensorValue(self.dms_port)
        rospy.loginfo("Sensor value at port %d: %f", self.dms_port, dms_sensor_reading)

        return dms_sensor_reading

    def scan_right(self,deg):
        response = setMotorTargetPositionCommand(10, theta_to_encoder(-deg,10))
        rospy.sleep(self.sleep_time*5)
        dms_sensor_reading = getSensorValue(self.dms_port)
        rospy.loginfo("Sensor value at port %d: %f", self.dms_port, dms_sensor_reading)
        return dms_sensor_reading
    
    def scan_front(self):
        response = setMotorTargetPositionCommand(10, theta_to_encoder(0,10))
        rospy.sleep(self.sleep_time*5)
        dms_sensor_reading = getSensorValue(self.dms_port)
        rospy.loginfo("Sensor value at port %d: %f", self.dms_port, dms_sensor_reading)
        return dms_sensor_reading

    def walk_forward(self):
        encoder_speeds = []
        response = setMotorTargetPositionSync(4, (1, 2, 3, 4), (512, 512, 512, 512))
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (1023, 1023+930, 1023, 1023+930))
        encoder_speeds.append(getMotorWheelSpeed(5))
        encoder_speeds.append(getMotorWheelSpeed(6))
        encoder_speeds.append(getMotorWheelSpeed(7))
        encoder_speeds.append(getMotorWheelSpeed(8))
        #self.odometry(encoder_speeds,self.state)

        #motor_speed = getMotorWheelSpeed(7)
        #motor_position = getMotorPositionCommand(7)
        #rospy.loginfo("Motor 7 Wheel speed: %d\nMotor 7 position: %d\nMotor response %d",motor_speed, motor_position,response)

        #rospy.sleep(1.8)
        
    def walk_forward_n_units(self, n):
        self.walk_forward()
        rospy.sleep(1.60)
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (0, 0, 0, 0))
        #if(self.scan_front()>2500):
            #response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (1023+830, 923, 1023+830, 923))
            #rospy.sleep(0.2)
        for x in range(n-1):
            self.walk_forward()
            rospy.sleep(1.55)
            rospy.loginfo("walk forward one more unit")
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (0, 0, 0, 0))
    
    def walk_forward_n_units_odom(self, n):
        self.walk_forward()
        for x in range(n-1):
            response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (0, 0, 0, 0))

    def recorded_wheel_speed_to_linear_velocity(self):
        #y=0.0268x +0.1079
        motor_speed_5 = getMotorWheelSpeed(5) 
        motor_speed_6 = getMotorWheelSpeed(6) 
        motor_speed_7 = getMotorWheelSpeed(7)
        motor_speed_8 = getMotorWheelSpeed(8) 
        motor_speeds = np.array([motor_speed_5,motor_speed_6-1023, motor_speed_7,motor_speed_8-1023]) * 0.0268 + 0.1079
        print("Linear Wheel Speeds:")
        print(motor_speeds)
        print('\n')

        # If moving linearly:
        N = 0
        avg = 0
        for i in range(len(motor_speeds)):
            if motor_speeds[i] > 20:
                pass
            else:
                N+=1
                avg+= motor_speeds[i]
        if N > 0:
            return avg/N
        else:
            print("Forward Linear Velocity could not be found")
            return False

        # If the robot is turning need the angular velocity and the radius of the turning circle

    def turn_right(self):
        response = setMotorTargetPositionSync(4, (1, 2, 3, 4), (512+112, 512-112, 512-112, 512+112))
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (1023, 1023, 1023, 1023))
        rospy.sleep(0.59)
        response = setMotorTargetPositionSync(4, (1, 2, 3, 4), (512, 512, 512, 512))
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (0, 0, 0, 0))
        #response = setMotorTargetPositionCommand(1,512)
        #response = setMotorTargetPositionCommand(2,512)
        #response = setMotorTargetPositionCommand(3,512)
        #response = setMotorTargetPositionCommand(4,512)

        #response = setMotorWheelSpeed(5,1023)
        #response = setMotorWheelSpeed(6,1023)
        #response = setMotorWheelSpeed(7,1023)
        #response = setMotorWheelSpeed(8,1023)
    
    def turn_left(self):
        response = setMotorTargetPositionSync(4, (1, 2, 3, 4), (512+112, 512-112, 512-112, 512+112))
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (1023+930, 1023+930, 1023+930, 1023+930))
        rospy.sleep(0.71)
        response = setMotorTargetPositionSync(4, (1, 2, 3, 4), (512, 512, 512, 512))
        response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (0, 0, 0, 0))

    def planning(self, map, end):
        # Start = [i,j,dir]
        # End = [i,j,dir]
        map.setCost(end[0], end[1], 1)
        neighbor_queue = [[[end[0], end[1]], 1]]

        self.set_costs(neighbor_queue, map)
        map.printCostMap()
        
        #actually tell robot to move
        
        
        while (not (self.i == end[0] and self.j == end[1])):
            dir = 1
            min = 1000
            for x in range(4):
                curr_cost = map.getNeighborCost(self.i, self.j, x+1)
                if (curr_cost != 0 and curr_cost < min and map.getNeighborObstacle(self.i, self.j, x+1) == 0):
                    dir = x+1
                    min = curr_cost
            self.move(dir)
            #print("move %d", dir)
       
        #check for final dir
        if (not (self.dir == end[2])):
            turn_count = self.check_dir(end[2])
            self.turn(turn_count)
    
    def set_costs(self, neighbor_queue, map):
        map.printCostMap()
        curr_element = neighbor_queue.pop(0)
        curr = curr_element[0]
        distance_from_end = curr_element[1]
        if (not (curr[0] == self.i and curr[1] == self.j)):
            distance_from_end += 1
            for x in range (4):
                if(map.getNeighborObstacle(curr[0], curr[1], x+1) == 0):
                    if (map.getNeighborCost(curr[0], curr[1], x+1) == 0):
                        map.setNeighborCost(curr[0], curr[1], x+1, distance_from_end)
                        neighbor_queue.append([self.next_loc(curr[0], curr[1], x+1), distance_from_end])
                #else 
        if(len(neighbor_queue)>0):
            self.set_costs(neighbor_queue, map)
        # create a structure with al points of grid
        # explore neighboring spots from the end, add 1 to their val
        # continue with next neighboring, add 2....
        # stop when you have gotten to the start
        # now trace back to the goal by choosing lowest value spots
        # yay.
    
    def next_loc(self,i,j,dir):
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
        return [end_i,end_j]

    def turn(self, turn_count):
        if (turn_count > 0):
            for x in range(turn_count):
                print("Turning Right")
                #self.turn_right()
        else:
            print("Turning Left")
            #self.turn_left()

    def move_test(self, i, j, dir):
        #start facing south - dir = 3
        # north - 1, east - 2, west - 4
        start_i = 0
        start_j = 0
        curr_dir = 3
        if (i - start_i > 0):
            self.walk_forward_n_units(i - start_i)
        if (j - start_j > 0):
            self.turn_left()
            curr_dir -=1
            self.walk_forward_n_units(j - start_j)
            turn_count = self.check_dir(curr_dir, dir)
            if(turn_count >0):
                self.turn_right()
            else:
                self.turn_left()
            
    def move(self, dir_next):
        # move one unit in a specific direction based on current orientation
        # north - 1, east - 2, south - dir, west - 4
        if(self.dir != dir_next):
            turn_count = self.check_dir(dir_next)
            self.turn(turn_count)
            self.dir = dir_next

        print("Walking forward")
        #self.walk_forward_n_units(1)
        new_location = self.next_loc(self.i,self.j,self.dir)
        self.i = new_location[0]
        self.j = new_location[1]
    
    def check_dir(self, dir_next):
        # turning right by 90 = 1, left by 90 = -1
        # right by 180 = 2, left by 180 = -2
        test_dir = self.dir
        turn_count = 0
        while(test_dir != dir_next):
            if (test_dir < 4):
                test_dir += 1
            else:
                test_dir = 1
            turn_count += 1
        if (turn_count > 2):
            turn_count = -1
        return turn_count

    def odometry(self, forward_vel, W, R, dt):
    # Inputs:
    # W is the angular velocity of the COM about the turning point
    # R is the radius from the COM to the the turning point
    # dt is the time span in which the control loop functions
    # Need the current state of the robot [x,y,theta] have stored in the class
    # The updates to the current state

        if R==0:
            # Moving straight (based on orientation tho)
            self.theta += 0
            self.x += np.cos(self.theta)*forward_vel*dt
            self.y += np.sin(self.theta)*forward_vel*dt

            pass
        else:
            # Turning
            self.theta += W * dt
            
            # Use the change in and the radius to compute the change in x and y
            self.x += np.cos(self.theta)*linear_velocity_in_x*dt
            self.y += np.sin(self.theta)*linear_velocity_in_y*dt

    def mapping(self):
        start = [0, 0, 3]
        #somehow keep track of places the robot has been
        exploration_queue = []
        curr = start
        map = map.EECSMap()
        map.clearObstacleMap()
        map.clearCostMap()
        #look around left forward and right
        left_value,front_value,right_value = self.full_scan()
        # Also turn left and look left for behind value
        self.turn_left()
        behind_value = self.scan_left()
        
        #add obstacles
        obstacle_threshold = 2000
        if(left_value > obstacle_threshold):
            dir = curr[2]
            if(curr[2] > 1):
                dir = curr[2] - 1
            else:
                dir = 4
            map.setObstacle(curr[0], curr[1], 1, dir)
        if(front_value > obstacle_threshold):
            map.setObstacle(curr[0], curr[1], 1, curr[2])
        if(right_value > obstacle_threshold):
            dir = curr[2]
            if(curr[2] < 4):
                dir = curr[2] + 1
            else:
                dir = 1
            map.setObstacle(curr[0], curr[1], 1, dir)
        #go in first direction that it can go

    def path_control(map, current_postion):
        # Inputs 
        # map: the obsticle map
        # current_position : [x,y,dir] in map coordinates
        
        # Step one: Given where you are in the map. What obsticles are around you in relative terms? 
        if (current_postion[2] -1) < 1: 
            left_turn = 4
        else:
            left_turn = current_postion[2]-1

        if (current_postion[2]+1) > 4:
            right_turn = 1
        else:
            right_turn = current_postion[2]+1
        
        dir_array = [left_turn,current_postion[2],right_turn]
        
        if (map.getNeighborObstacle(current_postion[0], current_postion[1], dir_array[0])):
            # True means there is an obsticle to the left
            # Move distance sensor towards the obsticle and check that it is within a threshold
            control.scan_left(90)
            # Corrective Behavior
        else:
            print("There is no obsticle to the left of the robot")

        if (map.getNeighborObstacle(current_postion[0], current_postion[1], dir_array[1])):
            # There is an obsticle to the front of the robot
            control.scan_front()
            # Corrective Behavior
        else:
            print("There is no obsticle to the front")

        if (map.getNeighborObstacle(current_postion[0], current_postion[1], dir_array[2])):
            # There is an obsticle to the right of the robot
            control.scan_right(90)
            # Corrective Behavior
        else:
            print("There is no obsticle to the right")

    

        
        

        # Step two: update postion in the world based on sensor vals
        # Step three: Make corrections to robot position

        pass
'''
    def move_one_unit(self):
        #have to correlate number of walks to distance
        walk()
        walk()
        walk()
        walk()

    def south(self):
        move_one_unit()

    def north(self)
        turn_180()
        move_one_unit()

    def west(self)
        turn_90_left()
        move_one_unit()

    def south(self)
        turn_90_right()
        move_one_unit()
'''       
 


control = controller()

map = map.EECSMap()
map.printObstacleMap()

control.i = int(input("What is start_i?"))
control.j = int(input("What is start_j?"))
control.dir = int(input("What is start_dir?"))

i_end = int(input("What is end_i?"))
j_end = int(input("What is end_j?"))
dir_end = int(input("What is end_dir?"))
control.planning(map, [i_end, j_end, dir_end]) 
