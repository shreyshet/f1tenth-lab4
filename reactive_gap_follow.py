#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

global prev_steering_angle
prev_steering_angle = 0
global t_prev
t_prev = 0.0
global prev_range
prev_range = []
class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.drive_msg = AckermannDriveStamped()
        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback) #None #TODO
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=10) #TODO
    	

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # global prev_range
        # smooth_range = 
        temp = np.array(ranges)
        temp2 = ranges
        # Viewing Angle
        forward_angle_min = -80
        forward_angle_max = 80
        ind_start 	= int((len(ranges)-1)*(forward_angle_min+180)/360)
        ind_end 	= int((len(ranges)-1)*(forward_angle_max+180)/360)

        # print(ind_start)
        # print(ind_end)
        # smoothing
        window = 3
        
        for i in range(len(temp)):
        	if i < window:
        		temp[i] = sum(temp[0:i+1])/(i+1)
        	elif i<(len(temp)-window):
        		temp[i] = sum(temp[i:i+window+1])/window
        	else:
        		temp[i] = sum(temp[i:])/((len(temp)-i)+1) 


        proc_ranges = temp[ind_start:ind_end+1]
        # proc_ranges = ranges[ind_start:ind_end+1]        
        # print(proc_ranges)
        return proc_ranges, ind_start, ind_end

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        # a = np.array(free_space_ranges)

        # print(type(free_space_ranges))
        b = [bool(i) for i in free_space_ranges]
        # print(b)
        r = max((list(y) for (x,y) in itertools.groupby((enumerate(b)),operator.itemgetter(1)) if x != 0), key=len)
        # print(r)
        min_maxgap_ind = r[0][0]
        # print(r[0][0])
        max_maxgap_ind = r[-1][0]
        # print(r[-1][0]) 
        return min_maxgap_ind, max_maxgap_ind
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        maxgap_val = np.array(ranges[start_i:end_i])
        # print(maxgap_val.max())
        best_pti = np.where(maxgap_val == maxgap_val.max())
        # print(maxgap_val[best_pti[0]])
        # print(best_pti)
        return best_pti[0]

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        # min_angle = data.angle_min
        # max_angle = data.angle_max
        # angle_inc = data.angle_increment
        proc_ranges, range_istart, range_iend = self.preprocess_lidar(ranges)#,min_angle,max_angle,angle_inc)
        global prev_steering_angle
        global t_prev
        #debug
        # print(np.shape(np.array(proc_ranges)))

        #Find closest point to LiDAR
        l = np.array(proc_ranges)
        min_l = l.min()
        max_l = l.max() 
        min_l_index = np.where(l == min_l)[0]

        print("min_l_index: (in deg)")
        print(((range_istart+min_l_index)*(360.0/len(ranges))-180.0))

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_rad = 10
        l_frsp = l.copy()
        # print(l_frsp)
        for i in range(len(l)):
        	if abs(min_l_index[0]-i)<bubble_rad:
        		l_frsp[i] = 0.0 
        		pass
        # print(l_frsp)
        #Find max length gap 
        min_maxgap_ind, max_maxgap_ind = self.find_max_gap(l_frsp)
        
        # print()
        # print(((range_istart+min_maxgap_ind)*(360.0/len(ranges))-180.0))
        # print(((range_istart+max_maxgap_ind)*(360.0/len(ranges))-180.0))
        #Find the best point in the gap 
        best_point_FOV = self.find_best_point(min_maxgap_ind,max_maxgap_ind,l_frsp)

        # print(best_point)
        best_point = best_point_FOV + range_istart
        # print(best_point)
        # print(len(ranges))
        best_steering_angle = ((best_point*(360.0/len(ranges)))-180.0)*np.pi/180.0

        # steering_angle = curr_steering_angle  # steering angle in radians

        max_steering_angle = rospy.get_param('/f1tenth_simulator/max_steering_angle')
        
        print("Steering Angle: (in deg)")
        print(best_steering_angle*180.0/np.pi)
        steering_angle = best_steering_angle
        # steering_angle = 0.0#sa_deg*np.pi/180

  #       if sa_deg>20.0:
  #       	sa_deg = 20
  #       elif sa_deg>15.0:
  #       	sa_deg = 15
  #   	elif sa_deg>10.0:
  #   		sa_deg = 10
  #   	elif sa_deg >5.0:
  #   		sa_deg = 5.0
		# elif sa_deg < -5.0:
		# 	sa_deg = -5.0
		# elif 
        	#     steering_angle = max_steering_angle
	        # else:
	        # 	steering_angle = -max_steering_angle

        # print(steering_angle*180/np.pi)
        # print(min_l)
        speed = 0.25
        # print(speed)
        if min_l < 0.6:
        	speed = 0.25*min_l/0.6      

        # dt = rospy.Time.now() - t_prev
        # time.pause()
        #Publish Drive message
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "laser"
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = speed
        self.drive_pub.publish(self.drive_msg)
        # prev_steering_angle = curr_steering_angle
        # t_prev = rospy.Time.now()

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
