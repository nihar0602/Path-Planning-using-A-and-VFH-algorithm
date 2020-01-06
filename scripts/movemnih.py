#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math 
from math import atan2
import numpy as np 

slope= 0
x = 0.0
y = 0.0 
yaw = 0.0
current_x = 0
current_y = 0
start_x = 0
start_y = 0
goal_x = 0
goal_y = 0
end_x = 0
end_y = 0
sensor_angle_lists=[]
min_cost_angle = 0
sector_=[]

#LAST CORRECTED: 10/13/19 08:30PM - UPDATE PATH with coordinates. 



A  = [ 0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

map = np.reshape(A, (20,18))


def map_2_sim(row,col):           #Will give the values from grid-map to co-ordinates

    x_c  = col - 8.5
    y_c  = 9.5 - row

    return (x_c,y_c)


def sim_2_map(x,y):             #Will give the values from co-ordinated to the grid-map
   
    row = int(np.floor(y))
    col = int(np.floor(x))

    r = 9 - row
    c = 9 + col 
    return (r,c)


goal_x = rospy.get_param('/goalx')
goal_y = rospy.get_param('/goaly')

end = sim_2_map(goal_x,goal_y)
start = sim_2_map(-8,-2)

print(end)
print(start)

# print (end)
# print (start)
# print (end[1])
# print (map[end[0]][end[1]])


def target_direction(b):
    global start_x, start_y, end_x, end_y,current_x,current_y
    t=slope_(current_x,current_y,end_x,end_y)
    
    if t<0 and end_x-current_x<0:
        t=math.pi+t
    if t>0 and end_x-current_x<0:
        t=math.pi-t
    x = abs(t - b)
    return x

def current_direction(a,b):
    x = abs(a - b)
    return x
    
def slope_(a,b,c,d):
    slope = math.atan2(d - b, abs(c - a))
    return slope

# def previous_direction(a,b):
#     x = abs(a-b)
#     return x 

def heuristic(node,end):
    h = (math.sqrt( (node[0]-end[0])**2 + (node[1]-end[1])**2 ))
    return h

def get_g_cost(node,current_node):
    gcost_ = math.sqrt( (node[0]-current_node[0])**2 + (node[1]-current_node[1])**2 )
    return gcost_


def get_cost(node,current_node):
    g_cost = current_node_g + (math.sqrt( (node[0]-current_node[0])**2 + (node[1]-current_node[1])**2 )) 
    h_cost = heuristic(node,end)
    f_cost = g_cost+h_cost

    return f_cost



def Astar(map,start,end):
    global current_node_g
    start_node = start
    goal_node = end


    open_list = []
    opencost_list = []
    open_g_cost_list = []
    closed_list = []
    current_node_g = 0

    open_list.append(start_node)
    opencost_list.append(get_cost(start_node,start))

    while len(open_list) > 0:

        current_node = open_list[opencost_list.index(min(opencost_list))]

        open_list.remove(current_node)
        opencost_list.remove(min(opencost_list))
        closed_list.append(current_node)

        if current_node == goal_node:
            print ('reached')
            return closed_list

        neighbours = [(0,-1),(0,0),(0,1),(-1,-1),(-1,0),(-1,1),(1,-1),(1,0),(1,1)]
        for next_new_node in neighbours:

            new_node = (current_node[0]+next_new_node[0],current_node[1]+next_new_node[1])
           
            if map[new_node[0]][new_node[1]] != 0:
                continue
            
            if new_node[0] > (len(map) - 1) or new_node[0] < 0 or new_node[1] > (len(map[len(map)-1]) -1) or new_node[1] < 0:
                continue

           
            if new_node in closed_list:
               continue

            open_list.append(new_node)   
            # new_node_g = get_g_cost(new_node,current_node)
            # print(new_node_g)
            cost = get_cost(new_node,current_node)               
            opencost_list.append(cost)


def distance_(a,b,c,d):
    dist_ = math.sqrt(pow(d- b, 2) + pow(c - a, 2))
    print(a,b,c,d)
    return dist_

# def slope_(a,b,c,d):
#     slope = math.atan2(d - b, c - a)
#     return slope


def callback_bin(msg):

    #CANDIDATE DIRECTION  = SENSOR_ANGLE_LIST[CURRENT INDEX OF THE LOOP THAT HAS GAP COST VALUE 0], i.e [i]
    
    vel_msg = Twist()
    global yaw, sector_
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
    L_12 = sector_[350:360]
    

    sensor_lists = [R_1,R_2,R_3,R_4,R_5,R_6,R_7,R_8,R_9,R_10,R_11,R_12,F_1,F_2,F_3,F_4,F_5,F_6,F_7,F_8,F_9,F_10,F_11,F_12,L_1,L_2,L_3,L_4,L_5,L_6,L_7,L_8,L_9,L_10,L_11,L_12]
    # sensor_angle_lists = [-1.48,-1.39,-1.30,-1.22,-1.13,-1.04,-0.95,-0.87,-0.78,-0.69,-0.61,-0.52,-0.43,-0.34,-0.26,-0.17,-0.08,0,0.08,0.17,0.26,0.34,0.43,0.52,0.61,0.69,0.78,0.87,0.95,1.04,1.134,1.22,1.30,1.39,1.48,1.57]
    sensor_angle_lists = [1.48,1.39,1.30,1.22,1.13,1.04,0.95,0.87,0.78,0.69,0.61,0.52,0.43,0.34,0.26,0.17,0.08,0,-0.08,-0.17,-0.26,-0.34,-0.43,-0.52,-0.61,-0.69,-0.78,-0.87,-0.95,-1.04,-1.134,-1.22,-1.30,-1.39,-1.48,-1.57]
    count_sensor_lines = []
    gap_values = []
    gap_cost_values = []
    a = 20
    b = 5
    c = 1 

    number = 0
    for sect_ in sensor_lists:
         number = 0
         for lasercount in sect_:
             if lasercount < 1:             #this tells us that if the sensor readings are less than 3 then it will count as a bin in the list.
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
            gap_cost_values.append(cost)
            min_cost_angle = sensor_angle_lists[gap_cost_values.index(min(gap_cost_values))]
        previous_direction = min_cost_angle
    
    
    # print('CSL',count_sensor_lines)
    # print('GAP_VAL',gap_values)
    # print('Index', gap_cost_values)

    # print("MIN COST ANGLEEEEEEss",min_cost_angle)
    # print("GAP COST VALUESSS",gap_cost_values)


def clb_odom(msg):
    global current_x, current_y 
    global yaw

    current = msg.pose.pose.position

    current_x = current.x
    current_y = current.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # print "The value of x and y and theta are",x,y,yaw,(yaw*180)/3.142
    # print("X:",current_x)
    # print("Y:",current_y)
    return

def main():
    global goal_x, goal_y
    global sensor_angle_lists, min_cost_angle
    global start_x, start_y, end_x, end_y
    global sector_



    rospy.init_node("ROS_PA_2")
    sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, clb_odom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    rospy.Subscriber('/base_scan', LaserScan,callback_bin)


    global current_x, current_y
    # init = rospy.wait_for_message('/base_pose_ground_truth',Odometry).pose.pose.position


    vel_msg = Twist()


    goal_x = rospy.get_param('/goalx')
    goal_y = rospy.get_param('/goaly')

    path = Astar(map,start,end)
    print(path)

    # print(map_2_sim(path[0][0], path[0][1]))

    coordinates_ = []

    for cord_ in path:
        coordinates_.append(map_2_sim(cord_[0], cord_[1]))

    # print coordinates_

    # print (coordinates_[0][0],coordinates_[0][1])

   
  

    while not rospy.is_shutdown():
        
        start_x = (coordinates_[0][0])
        start_y = (coordinates_[0][1])
        
        r = rospy.Rate(5)

        print(start_x,start_y)

        end_x = (coordinates_[1][0]) 
        end_y = (coordinates_[1][1])

        print(end_x,end_y)
       
        

        while distance_(current_x,current_y,end_x,end_y) >  0.45:
            # print("FFFFFFFFFFFFFFFFFf",sector_[:])
            

            if (yaw) > (yaw - min_cost_angle):
                vel_msg.linear.x = 0.3
                vel_msg.angular.z = -0.4  
                pub.publish(vel_msg)
            if (yaw) < (yaw - min_cost_angle):
                vel_msg.linear.x = 0.3
                vel_msg.angular.z = 0.4
                pub.publish(vel_msg)

            # if abs((yaw) - (yaw - min_cost_angle)) > 0.01:
            #     vel_msg.linear.x = 0.0
            #     vel_msg.angular.z = -0.4  
            #     pub.publish(vel_msg)

            # if abs((yaw) - (yaw - min_cost_angle)) > -0.01:
            #     vel_msg.linear.x = 0.0
            #     vel_msg.angular.z = 0.4  
            #     pub.publish(vel_msg)
            # else:
            #     vel_msg.linear.x = 0.4
            #     vel_msg.angular.z = 0.0
            #     pub.publish(vel_msg)
            
            if len(sector_) > 10:
                if min(sector_[:]) < 0.5:
                    print("TRRRRRRRRRRRRREWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE")
                    # vel_msg.linear.x = 0.1
                    vel_msg.angular.z = -1.5
                    pub.publish(vel_msg)
                    
            vel_msg.linear.x = 0.15
            pub.publish(vel_msg)
           
            r.sleep()
    
        if distance_(current_x,current_y,end_x,end_y) < 0.45:
            coordinates_.pop(0)
        
            r.sleep()

        print(coordinates_)
        print("Distance:",distance_(current_x,current_y,end_x,end_y) > 0.2)

            
        # if abs(slope - yaw) > 0.1:
        #     vel_msg.linear.x = 0.0
        #     vel_msg.angular.z = -0.3    
        # else:
        #     vel_msg.linear.x = 0.5
        #     vel_msg.angular.z = 0.0 
        r.sleep() 
        

    # pub.publish(vel_msg)
       


if __name__ == "__main__":
    main()
    pass