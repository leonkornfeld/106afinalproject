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

# ## CHANGES FOR LINEAR TRAJECTORY
# from sawyer_full_stack.src.paths.trajectories import LinearTrajectory
# from sawyer_full_stack.src.paths.paths import MotionPath
# from sawyer_full_stack.src.paths.path_planner import PathPlanner
# import intera_interface
# from trac_ik_python.trac_ik import IK
# from moveit_msgs.msg import DisplayTrajectory, RobotState



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

def sweep(request, compute_ik, x, y, z):
    request.ik_request.pose_stamped.pose.position.x = x
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = z 

    # facing down : 1.5 0 -0.5 1.7 from custom_sawyer_tuck.launch  
    request.ik_request.pose_stamped.pose.orientation.x = -0.016
    request.ik_request.pose_stamped.pose.orientation.y = 0.707
    request.ik_request.pose_stamped.pose.orientation.z = -0.018
    request.ik_request.pose_stamped.pose.orientation.w = 0.707

    # print(request)

    response = compute_ik(request)
    # print(response)

    group = MoveGroupCommander("right_arm")
    group.set_pose_target(request.ik_request.pose_stamped)
    group.set_max_velocity_scaling_factor(0.5)
    plan = group.plan()
    group.execute(plan[1])
    rospy.sleep(1)

    #TODO: linear trajectory for sweep
    #TODO: tune the final orientation so that the ar tags show up in rviz
    #TODO: make it faster similar to how we did in move



def move(request, compute_ik, x,y,z, close):
    right_gripper = robot_gripper.Gripper('right_gripper')
    request.ik_request.pose_stamped.pose.position.x = x
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = z      
    # -0.008, 0.632, -0.101, 0.768
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
    group.set_max_velocity_scaling_factor(.5)

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

    number_of_blocks = 7 #change this value every time change number of blocks
    temp_ar_tag_index = number_of_blocks

    ar_tags_to_sort = [i for i in range(number_of_blocks)]
    rospy.wait_for_service('compute_ik')

    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        right_gripper = robot_gripper.Gripper('right_gripper')
        right_gripper.open()
        input('Starting sort - Press [ Enter ]: ')
        pos_list = [] #list of ar tag coordinates (x, y, z)
        position_to_index = {} #dictionary of position (x,y,z) -> ar tag index (currently 0 -> 3)
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = "base"

        for tag in ar_tags_to_sort:
            if tag not in position_to_index.values():
                try:
                    trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag}", rospy.Time(0), rospy.Duration(10.0))
                    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
                    pos_list.append(tuple(tag_pos))
                    position_to_index[tuple(tag_pos)] = tag
                except Exception as e:
                    print(e)
                    print("Retrying ...")

        assert(len(pos_list) > 0) 
        if len(position_to_index.values()) != number_of_blocks:
            # move to a found ar tag -> sweep!
            print("Sweeping since missing ar tags")
            sweep(request, compute_ik, pos_list[0][0], pos_list[0][1], 0)
            # move(request, compute_ik, pos_list[0][0], pos_list[0][1], 0, 2) # move to an arbitrary ar tag TODO: maybe don't have it be arbitrary
            epsilon = 0.0
            while len(position_to_index.values()) != number_of_blocks: 
                epsilon += 0.1
                print("Performing sweep in direction 1")
                sweep(request, compute_ik, pos_list[0][0], pos_list[0][1] + epsilon, 0.4)
                for tag in ar_tags_to_sort:
                    if tag not in position_to_index.values():
                        try:
                            trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag}", rospy.Time(0), rospy.Duration(10.0))
                            tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
                            pos_list.append(tuple(tag_pos))
                            position_to_index[tuple(tag_pos)] = tag
                        except Exception as e:
                            print(e)
                            print("Retrying ...")
                print("Performing sweep in direction 2")
                sweep(request, compute_ik, pos_list[0][0], pos_list[0][1] - epsilon, 0.4)
                for tag in ar_tags_to_sort:
                    if tag not in position_to_index.values():
                        try:
                            trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag}", rospy.Time(0), rospy.Duration(10.0))
                            tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
                            pos_list.append(tuple(tag_pos))
                            position_to_index[tuple(tag_pos)] = tag
                        except Exception as e:
                            print(e)
                            print("Retrying ...")

                input(f'Continue sweeping / done sweeping - Press [ Enter ]')


        
        # print('unsorted pos_list')
        # for pos in pos_list: print(pos_dict[pos])
        pos_list.sort(key=lambda triple: triple[1]) # sort by y-value instead of x-value since that corresponds to location in base_frame
        # print("sorted position list is ")
        # for pos in pos_list: print(pos_dict[pos])
        # print("position dict is ", pos_dict)
        initial_value_order = []
        index_to_position = {} #reversed pos dict index -> xyz
        for pos in pos_list:
            index_to_position[len(initial_value_order)] = pos
            initial_value_order.append(position_to_index[pos])
        #print(initial_value_order)
        trans = tfBuffer.lookup_transform("base", f"ar_marker_{temp_ar_tag_index}", rospy.Time(0), rospy.Duration(10.0))
        temp_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
        index_to_position[temp_ar_tag_index] = tuple(temp_pos)
        print("initial value order", initial_value_order)
        selection_sort_visualization = selection_sort(initial_value_order)
        print("list after selection sort ", selection_sort_visualization)
        print("ind dictionary", index_to_position)
        input("Start moving! Press [ Enter ]")
        
        for entry in selection_sort_visualization:
            try:
                print("entry is ", entry)
                input('Move to next block - Press [ Enter ]')
                print(f'picking up from {entry.start}')
                move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], 0, 2)
                move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], -.15, 1) # 1 means close
                move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], 0, 2)
                print(f'placing at {entry.end}')
                move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], 0, 2)
                move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], -.15, 0) # 0 means open
                move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], 0, 2)
                
                
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()

