#!/usr/bin/env python
import roslib
import rospy
import numpy as np
from fw_wrapper.srv import *
from map import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set multiple motors' speeds synchronously
def setMotorWheelSpeedSync(number_of_ids, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', 0, 0, number_of_ids, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set multiple motors' target positions synchronously
def setMotorTargetPositionSync(number_of_ids, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPositionsSync', 0, 0, number_of_ids, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def encoder_to_theta(encoder_count):
    theta = encoder_count-512
    theta = theta * 360/1024
    return theta

def theta_to_encoder(theta, motorId):
    encoder_count = theta*1024/360
    if(motorId == 1 or motorId == 4 or motorId == 5 or motorId == 8):
        encoder_count = -encoder_count
    encoder_count = encoder_count + 512
    return encoder_count


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
        dms_readings = []
        start_motor_position = 300
        end_position = 512-300+512
        N = 30
        for i in np.linspace(start_motor_position,end_position,30):
            response = setMotorTargetPositionCommand(10, i)
            rospy.sleep(self.sleep_time*5)
            dms_readings.append(getSensorValue(self.dms_port))
        
    def scan_left(self,deg):
        # Assuming that once the motor achieves the commanded motor position it will return true
        # TODO: Need to check this assumption, and integrate into the other scan functions
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

    def k_nearest_neighbors(self):
        
        pass
 
        
# 15cm DMS sensor reading 1350

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # control loop running at 1000hz
    r = rospy.Rate(1000) # 1000hz
    dms_port = 6
    # ir_port = 2
    dms_value_threshold = 600
    # ir_value_threshold = 10
    dt = 0

    control = controller()

    map = EECSMap()
    map.printObstacleMap()
    
    control.i = int(input("What is start_i?"))
    control.j = int(input("What is start_j?"))
    control.dir = int(input("What is start_dir?"))

    i_end = int(input("What is end_i?"))
    j_end = int(input("What is end_j?"))
    dir_end = int(input("What is end_dir?"))
    control.planning(map, [i_end, j_end, dir_end]) 
    
    '''
    response = setMotorWheelSpeed(5,1023)
    response = setMotorWheelSpeed(6,1023+930)
    response = setMotorWheelSpeed(7,1023)
    response = setMotorWheelSpeed(8,1023+930)
    '''
    #control.planning(map, [0, 0, 3], [4, 4, 3])
    #control.move_test(2, 2, 3)
    #control.turn_left()
    #control.home()
    #control.walk_forward()
    #control.walk_forward_n_units(1)
    #response = setMotorWheelSpeedSync(4, (5, 6, 7, 8), (1023, 1023+930, 1023, 1023+930))
    #motor_speed = getMotorWheelSpeed(7)
    #motor_position = getMotorPositionCommand(7)
    #rospy.loginfo("Motor 7 Wheel speed: %d\nMotor 7 position: %d\nMotor response %d",motor_speed, motor_position,response)
    #dt = 0
    while not rospy.is_shutdown():
        '''
        forward_vel = control.recorded_wheel_speed_to_linear_velocity()
        if forward_vel:
            control.odometry(forward_vel,R=0,W=0,dt=dt)
        '''
        r.sleep()
        #dt = 0.001
        #print(dt)
 
