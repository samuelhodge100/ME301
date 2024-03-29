#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import csv
from fw_wrapper.srv import *
from map import *
from collections import Counter
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches


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
            dms_sensor_reading = getSensorValue(self.dms_port)
            rospy.loginfo("Robot scan pos %d, val %f", i, dms_sensor_reading)
            dms_readings.append(dms_sensor_reading)
        response = setMotorTargetPositionCommand(10, start_motor_position)
        rospy.sleep(self.sleep_time*7)
        f = open("convex_57D_22cm_wall_data.csv", "a")
        for i in dms_readings:
            f.write(str(i) + ",")
        f.write("\n")
        f.close()

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
    
    def test_scan(self):
        dms_readings = []
        start_motor_position = 300
        end_position = 512-300+512
        N = 30
        
        for i in np.linspace(start_motor_position,end_position,30):
            response = setMotorTargetPositionCommand(10, i)
            rospy.sleep(self.sleep_time*5)
            dms_sensor_reading = getSensorValue(self.dms_port)
            rospy.loginfo("Robot scan pos %d, val %f", i, dms_sensor_reading)
            dms_readings.append(dms_sensor_reading)
        response = setMotorTargetPositionCommand(10, start_motor_position)
        rospy.sleep(self.sleep_time*7)
        knn = K_Nearest_Neighbors()
        knn.fit_data()
        prediction = knn.predict(dms_readings)
        rospy.loginfo("Predicted walltype is %s", prediction)

class K_Nearest_Neighbors:
    def __init__(self):
        # k between 1 and 20
        self.k = 3
        self.classes = ["concave", "convex", "straight_wall"]
        self.datafiles = [
            ["concave_57D_11cm_wall_data.csv", "concave"],
            ["concave_57D_16cm_wall_data.csv", "concave"],
            ["concave_57D_22cm_wall_data.csv", "concave"],
            ["convex_57D_11cm_wall_data.csv", "convex"],
            ["convex_57D_16cm_wall_data.csv", "convex"],
            ["convex_57D_22cm_wall_data.csv", "convex"],
            ["straight_wall_11cm_data.csv", "straight_wall"],
            ["straight_wall_16cm_data.csv", "straight_wall"],
            ["straight_wall_22cm_data.csv", "straight_wall"],
        ]
        self.x_data = []
        self.class_data = []

    def normalize_dataset(self, dataset):
        # Normalize the dataset
        norm_dataset = []
        for row in dataset:
            norm_row = np.linalg.norm(row)
            norm_dataset.append([r / norm_row for r in row])
        return norm_dataset

    def fit_data(self):
        # sets up x_data and class_data
        dataset = self.read_data()
        self.x_data = self.normalize_dataset(dataset)

    def two_vec_dist(self, row1, row2):
        # Compute the distance between two vectors
        return np.linalg.norm(np.array(row1) - np.array(row2))

    def predict(self, predict_data):
        # Takes in an unknown array of scan data and returns a classification
        norm = np.linalg.norm(predict_data)
        norm_predict_data = [p / norm for p in predict_data]
        distances = []
        for x in range(len(self.x_data)):
            distances.append([self.two_vec_dist(norm_predict_data, self.x_data[x]), x])
        distances.sort()
        predictions = []
        for x in range(self.k):
            predictions.append(self.class_data[distances[x][1]])
        print(predictions)
        prediction = Counter(predictions).most_common(1)[0][0]
        print(prediction)
        self.plot_data_predicted(norm_predict_data)
        return prediction

    def read_data(self):
        # Put data from csv file into a dataset
        dataset = []
        class_dataset = []
        for d, c in self.datafiles:
            f = open(d, "r")
            row = f.readline()
            while row:  # read until end of file
                row_data = row.split(",")
                # last element might be an empty string
                row_data.pop()
                row_data = list(map(int, row_data))
                dataset.append(row_data)
                class_dataset.append(c)
                row = f.readline()
            f.close()
        self.class_data = class_dataset
        return dataset

    def plot_data(self):
        fig, ax = plt.subplots(1)
        for i in range(len(self.x_data)):
            if (self.class_data[i] == 'concave'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='r',marker='o',label='concave')
            elif(self.class_data[i] =='convex'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='b',marker='^',label='covnvex')
            elif(self.class_data[i] =='straight_wall'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='g',marker='*',label='straight_wall')
        concave_series = mpatches.Patch(color='red', label='concave')
        convex_series = mpatches.Patch(color='blue', label='convex')
        straight_wall_series = mpatches.Patch(color='green', label='straight_wall')
        ax.set_title('Normalized Dataset')
        ax.set_xlabel('Index')
        ax.set_ylabel('Normalized DMS Sensor Reading')
        ax.legend(handles=[concave_series,convex_series,straight_wall_series])
        plt.show()
    
    def plot_data_predicted(self, data_set):
        fig, ax = plt.subplots(1)
        for i in range(len(self.x_data)):
            if (self.class_data[i] == 'concave'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='r',marker='o',label='concave')
            elif(self.class_data[i] =='convex'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='b',marker='^',label='convex')
            elif(self.class_data[i] =='straight_wall'):
                ax.scatter(range(len(self.x_data[i])),self.x_data[i],5,color='g',marker='*',label='straight_wall')
        ax.scatter(range(len(data_set)), data_set,25,color='black',marker='x',label='prediction')
        concave_series = mpatches.Patch(color='red', label='concave')
        convex_series = mpatches.Patch(color='blue', label='convex')
        straight_wall_series = mpatches.Patch(color='green', label='straight_wall')
        prediction_series = mpatches.Patch(color='black', label='prediction')
        ax.set_title('Normalized Dataset with Prediction')
        ax.set_xlabel('Index')
        ax.set_ylabel('Normalized DMS Sensor Reading')
        ax.legend(handles=[concave_series,convex_series,straight_wall_series,prediction_series])
        plt.show()
 
        
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

    control.test_scan()
    #knn = K_Nearest_Neighbors()
    #knn.fit_data()
    #knn.plot_data()
    #for i in range(20):
        #control.full_scan()

    while not rospy.is_shutdown():
        '''
        forward_vel = control.recorded_wheel_speed_to_linear_velocity()
        if forward_vel:
            control.odometry(forward_vel,R=0,W=0,dt=dt)
        '''
        r.sleep()
        #dt = 0.001
        #print(dt)
 
