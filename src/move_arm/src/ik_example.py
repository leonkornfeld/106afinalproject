#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
import tf2_ros


print('Done!')



class Entry():
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def __repr__(self):
        return f"Entry(start={self.start}, end={self.end})"

def selection_sort(arr):
    temp = len(arr)
    output_list = []
    # Traverse through all elements in the array
    for i in range(len(arr)):
        # Find the minimum element in the unsorted portion
        min_index = i
        for j in range(i + 1, len(arr)):
            if arr[j] < arr[min_index]:
                min_index = j
        if i != min_index:
            output_list.append(Entry(min_index, temp))
            output_list.append(Entry(i, min_index))
            output_list.append(Entry(temp, i))
        arr[i], arr[min_index] = arr[min_index], arr[i]

    return output_list


def move(request, compute_ik, x,y,z, close):
    right_gripper = robot_gripper.Gripper('right_gripper')
    request.ik_request.pose_stamped.pose.position.x = x
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = z      
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    #user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    
    group.execute(plan[1])
    if close == 0:
        right_gripper.open()
    elif close == 1:
        right_gripper.close()
    rospy.sleep(1)
    
def main():
    # Wait for the IK service to become available

    rospy.wait_for_service('compute_ik')

    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        right_gripper = robot_gripper.Gripper('right_gripper')

        # Calibrate the gripper (other commands won't work unless you do this first)
       # print('Calibrating...')
        #right_gripper.calibrate()
        #rospy.sleep(2.0)

        # Close the right gripper
        #print('Closing...')
        #right_gripper.close()
        #rospy.sleep(1.0)

        # Open the right gripper
        #print('Opening...')
        #right_gripper.open()
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        
        l = [0,6,8]
        pos_list = []
        pos_dict = {}
        try:
            # TODO: lookup the transform and save it in trans
            # The rospy.Time(0) is the latest available 
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
            for tag in l:
                trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag}", rospy.Time(0), rospy.Duration(10.0))
                tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
                pos_list.append(tuple(tag_pos))
                pos_dict[tuple(tag_pos)] = tag
        except Exception as e:
            print(e)
            print("Retrying ...")
                
        
        pos_list.sort()
        print(pos_list)
        print(pos_dict)
        initial_value_order = []
        ind_dct = {}
        for pos in pos_list:
            ind_dct[len(initial_value_order)] = pos
            initial_value_order.append(pos_dict[pos])
        #print(initial_value_order)
        trans = tfBuffer.lookup_transform("base", "ar_marker_2", rospy.Time(0), rospy.Duration(10.0))
        temp_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        ind_dct[len(initial_value_order)] = tuple(temp_pos)
        l = selection_sort(initial_value_order)
        print(l)
        print(ind_dct)
        input()
        
        for entry in l:
            try:
                print(entry)
                input()
                move(request, compute_ik, ind_dct[entry.start][0], ind_dct[entry.start][1], 0, 2)
                move(request, compute_ik, ind_dct[entry.start][0], ind_dct[entry.start][1], -.15, 1)
                move(request, compute_ik, ind_dct[entry.start][0], ind_dct[entry.start][1], 0, 2)
                move(request, compute_ik, ind_dct[entry.end][0], ind_dct[entry.end][1], 0, 2)
                move(request, compute_ik, ind_dct[entry.end][0], ind_dct[entry.end][1], -.15, 0)
                move(request, compute_ik, ind_dct[entry.end][0], ind_dct[entry.end][1], 0, 2)
                
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
