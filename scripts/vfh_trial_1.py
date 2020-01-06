#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

yaw = 0.0   
sensor_angle_lists=[]
min_cost_angle = 0
start_x = -8
start_y = -2
end_x = -7
end_y = -1
mode = 0



def target_direction(b):
    global start_x, start_y, end_x, end_y
    x = abs(slope_(start_x,start_y,end_x,end_y) - b)
    return x

def current_direction(a,b):
    x = abs(a - b)
    return x
    
def slope_(a,b,c,d):
    slope = math.atan2(d - b, c - a)
    return slope

# def previous_direction(a,b):
#     x = abs(a-b)
#     return x 


def slope_(a,b,c,d):
    slope = math.atan2(d - b, c - a)
    return slope


def clb_odom(msg):
    global roll, pitch, yaw
    global current

    current = msg.pose.pose.position

    current_x = current.x 
    current_y = current.y 

    euler_orientation = msg.pose.pose.orientation
    quaternion = [euler_orientation.x, euler_orientation.y, euler_orientation.z, euler_orientation.w]
    (roll, pitch, yaw)= euler_from_quaternion(quaternion)
      #print 'Euler Y = [%s]' %yaw

def callback_bin(msg):

    #CANDIDATE DIRECTION  = SENSOR_ANGLE_LIST[CURRENT INDEX OF THE LOOP THAT HAS GAP COST VALUE 0], i.e [i]
    
    vel_msg = Twist()
    global yaw
    global sensor_angle_lists, min_cost_angle

    sector_ = list(msg.ranges)
    R_1 = sector_[0:10]
    R_2 = sector_[10:20]
    R_3 = sector_[20:30]
    R_4 = sector_[30:40]
    R_5 = sector_[40:50]
    R_6 = sector_[50:60]
    R_7 = sector_[60:70]
    R_8 = sector_[70:80]
    R_9 = sector_[80:90]
    R_10 = sector_[90:100]
    R_11 = sector_[100:110]
    R_12 = sector_[110:120]
    F_1 = sector_[120:130]
    F_2 = sector_[130:140]
    F_3 = sector_[140:150]
    F_4 = sector_[150:160]
    F_5 = sector_[160:170]
    F_6 = sector_[170:180]
    F_7 = sector_[180:190]
    F_8 = sector_[190:200]
    F_9 = sector_[200:210]
    F_10 = sector_[210:220]
    F_11 = sector_[220:230]
    F_12 = sector_[230:240]
    L_1 = sector_[240:250]
    L_2 = sector_[250:260]
    L_3 = sector_[260:270]
    L_4 = sector_[270:280]
    L_5 = sector_[280:290]
    L_6 = sector_[290:300]
    L_7 = sector_[300:310]
    L_8 = sector_[310:320]
    L_9 = sector_[320:330]
    L_10 = sector_[330:340]
    L_11 = sector_[340:350]
    L_12 = sector_[350:361]
    

    sensor_lists = [R_1,R_2,R_3,R_4,R_5,R_6,R_7,R_8,R_9,R_10,R_11,R_12,F_1,F_2,F_3,F_4,F_5,F_6,F_7,F_8,F_9,F_10,F_11,F_12,L_1,L_2,L_3,L_4,L_5,L_6,L_7,L_8,L_9,L_10,L_11,L_12]
    # sensor_angle_lists = [-1.48,-1.39,-1.30,-1.22,-1.13,-1.04,-0.95,-0.87,-0.78,-0.69,-0.61,-0.52,-0.43,-0.34,-0.26,-0.17,-0.08,0,0.08,0.17,0.26,0.34,0.43,0.52,0.61,0.69,0.78,0.87,0.95,1.04,1.134,1.22,1.30,1.39,1.48,1.57]
    sensor_angle_lists = [1.48,1.39,1.30,1.22,1.13,1.04,0.95,0.87,0.78,0.69,0.61,0.52,0.43,0.34,0.26,0.17,0.08,0,-0.08,-0.17,-0.26,-0.34,-0.43,-0.52,-0.61,-0.69,-0.78,-0.87,-0.95,-1.04,-1.134,-1.22,-1.30,-1.39,-1.48,-1.57]
    count_sensor_lines = []
    gap_values = []
    gap_cost_values = []
    a = 3.0
    b = 1.0
    c = 1.0 

    number = 0
    for sect_ in sensor_lists:
         number = 0
         for lasercount in sect_:
             if lasercount < 3:             #this tells us that if the sensor readings are less than 3 then it will count as a bin in the list.
                number = number + 1

         count_sensor_lines.append(number)

    for gapvalues in count_sensor_lines:
        if gapvalues > 4:                               #the value 4 represents if the no. of obstacles in that bin is greater than four. 
            gap_val = 1
        else:
            gap_val = 0

        gap_values.append(gap_val)

    previous_direction = yaw
   
    for i,find_costs in enumerate(gap_values):
        if find_costs == 1:
            cost = 9999999
            gap_cost_values.append(cost)
        elif find_costs == 0:
            # cost = sensor_angle_lists[i]
            # cost = (a*target_direction(sensor_angle_lists[i])) + (b*current_direction(sensor_angle_lists[i],yaw)) + (c*previous_direction(yaw,sensor_angle_lists[i]))
            cost = (a*target_direction(yaw - sensor_angle_lists[i])) + (b*current_direction(yaw,(yaw - (sensor_angle_lists[i])))) + (c* (abs(previous_direction - (yaw-sensor_angle_lists[i])))) 
            # print("COSTTTTTT:",cost)
            gap_cost_values.append(cost)

            min_cost_angle = sensor_angle_lists[gap_cost_values.index(min(gap_cost_values))]
        previous_direction = min_cost_angle
        

    # print('CSL',count_sensor_lines)
    # print('GAP_VAL',gap_values)
    # print('Index', gap_cost_values)
 

    
  
    print("MIN COST ANGLEEEEEEss",min_cost_angle)
    
    # print("GAP COST VALUESSS",gap_cost_values)
    

   

    
            



     

def main():
    global sensor_angle_lists, min_cost_angle, mode
    global start_x, start_y
    mode = 0
    rospy.init_node('PA_21')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=[1])
    rospy.Subscriber ('/base_pose_ground_truth', Odometry, clb_odom)
    rospy.Subscriber('/base_scan', LaserScan,callback_bin)
    
    
    vel_msg = Twist()

    

    x = rospy.get_param('/goalx')
    y = rospy.get_param('/goaly')

    print(x)
    print(y)

    while not rospy.is_shutdown():
        r = rospy.Rate(5)

        slope = slope_(start_x,start_y,end_x,end_y)
        print(slope)

        if abs(yaw - (yaw - min_cost_angle)) > 0.01:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.3  
            pub.publish(vel_msg)
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            pub.publish(vel_msg)
           
        print("MIN COST ANGLE:",min_cost_angle)   
        print("YAAWWWWWWWw",yaw)
        print("DEKHOOOOOOO",abs(yaw - (yaw- min_cost_angle)) > 0.01)
        
        # print("MODE:", mode)
        # if mode == 0: 
        #     if abs(yaw - min_cost_angle) > 0.05:
        #         print("hiiiiiii")
        #         vel_msg.linear.x = 0.0
        #         vel_msg.angular.z = -0.3
        #         # print('YOOOOO', abs(yaw - min_cost_angle))
        #         pub.publish(vel_msg)
        #     if abs(yaw - min_cost_angle) < 0.07:
        #         mode = 1
        #     # else:
        #     #     vel_msg.linear.x = 0.3
        #     #     vel_msg.angular.z = 0.0
        #     #     pub.publish(vel_msg)
        #     #     mode = 1        

        # if mode == 1:
        #     print("MODE 1")
        #     if abs(yaw - min_cost_angle) > 0.05:
        #         vel_msg.linear.x = 0.5
        #         vel_msg.angular.z = 0.0
        #         pub.publish(vel_msg)
        #     # if abs(yaw - min_cost_angle) < 0.07: 
        #         # mode = 0
        #     # else:
        #     #     vel_msg.linear.x = 0.3
        #     #     vel_msg.angular.z = 0.0
        #     #     pub.publish(vel_msg)
       

        
        r.sleep()
    
  


if __name__ == "__main__":
    main()
    pass