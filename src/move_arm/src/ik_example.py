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
import math

from paths.trajectories import LinearTrajectory # pyright: ignore
from paths.paths import MotionPath # pyright: ignore
from paths.path_planner import PathPlanner # pyright: ignore
import intera_interface
from trac_ik_python.trac_ik import IK
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics # pyright: ignore

from color_detection import capture_image, get_color_order
from controllers.controllers import PIDJointVelocityController #pyright: ignore



print('Done!')

index_to_position = {} #reversed pos dict index -> xyz

class Entry():
    def __init__(self, start, end, level = None):
        self.start = start
        self.end = end
        self.level = level
    def __repr__(self):
        return f"Entry(start={self.start}, end={self.end}, level={self.level})"


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

def insertion_sort(arr):
    temp = len(arr)
    output_list = []
    for i in range(1, len(arr)):
        key = arr[i]  # element to be placed correctly
        j = i - 1
        output_list.append(Entry(j + 1, temp))
        flag = False
        while j >= 0 and arr[j] > key:
            flag = True
            output_list.append(Entry(j,j+1))
            arr[j + 1] = arr[j]
            j -= 1

        arr[j + 1] = key  # Place key after the last moved element
        if flag:
            output_list.append(Entry(temp, j+1))
        else:
            output_list.pop()
    return output_list

def merge_sort(arr, movements):
    """
    Perform merge sort on a list.
    
    :param arr: List of elements to be sorted
    :return: Sorted list
    """
    if len(arr) <= 1:
        print(arr)
        # movements.append(Entry(arr[0][1],arr[0][1]))
        return arr

    # Split the array into two halves
    mid = len(arr) // 2
    left_half = arr[:mid]
    right_half = arr[mid:]

    # Recursively sort both halves
    left_sorted = merge_sort(left_half, movements)
    right_sorted = merge_sort(right_half, movements)

    # Merge the sorted halves
    return merge(left_sorted, right_sorted, movements)

def merge(left, right, movements):
    """
    Merge two sorted lists into a single sorted list.
    
    :param left: First sorted list
    :param right: Second sorted list
    :return: Merged sorted list
    """
    merged = []
    left_index = right_index = 0

    # Merge while there are elements in both lists
    next = min(left[0][1], right[0][1])
    while left_index < len(left) and right_index < len(right):
        if left[left_index][0] < right[right_index][0]:
            # if next != left[left_index][1]:
            movements.append(Entry(left[left_index][1],next, math.ceil(math.log2((len(right))))))
            merged.append((left[left_index][0], next))
            left_index += 1

        else:
            # if next != right[right_index][1]:
            movements.append(Entry(right[right_index][1],next, math.ceil(math.log2((len(right))))))
            merged.append((right[right_index][0], next))
            right_index += 1
        next+=1
    

    # Add remaining elements from left and right
    if left_index == len(left):
        while right_index < len(right):
            # if next != right[right_index][1]:
            movements.append(Entry(right[right_index][1],next, math.ceil(math.log2((len(right))))))
            merged.append((right[right_index][0], next))
            right_index+=1
            next+=1
    elif right_index == len(right):
        while left_index < len(left):
            merged.append((left[left_index][0], next))
            # if next != left[left_index][1]:
            movements.append(Entry(left[left_index][1], next, math.ceil(math.log2((len(right))))))
            left_index+=1
            next+=1    
    # movements.append(Entry("split","split"))
    print(merged)
    return merged

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

    group.execute(plan[1])
    rospy.sleep(1)

    #TODO: linear trajectory for sweep
    #TODO: tune the final orientation so that the ar tags show up in rviz
    #TODO: make it faster similar to how we did in move

def get_trajectory(limb, kin, ik_solver, target_pos, num_waypoints=5):
    num_way = num_waypoints
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(1.0))
    except Exception as e:
        print(e)

    current_pos = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    # index_to_position[ind] = tuple(current_pos)


    trajectory = LinearTrajectory(start_position=current_pos, goal_position=target_pos, total_time=2)

    path = MotionPath(limb, kin, ik_solver, trajectory)

    # print("HELLLO", path.to_robot_trajectory(num_way, True))
    
    return path.to_robot_trajectory(num_way, True)

def linear_trajectory_move(ind, x, y, z, close=None):
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    
    target_pos = np.array([x, y, z])

    robot_trajectory = get_trajectory(limb, kin, ik_solver, target_pos)

#     planner = PathPlanner('right_arm')
#     previous = np.array([10,10,10,10,10,10,10])
#     # robot_trajectory.joint_trajectory.points = robot_trajectory.joint_trajectory.points[:-2]
#     for i in range(len(robot_trajectory.joint_trajectory.points)):
#         ##THIS GUY IS A PROBLEM (ignores z podition in first down motion somtimes) so changed from .6 to .1
#         if np.linalg.norm(previous - robot_trajectory.joint_trajectory.points[i].positions) < .03:
#             continue
#         # print(robot_trajectory.joint_trajectory.points[i])
#         # input()
#         plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[i].positions)
#         previous = robot_trajectory.joint_trajectory.points[i].positions
#         # import pdb;
#         # pdb.set_trace()
#         import time
#         t = time.time()
#         print("before execute plan")
#         planner.execute_plan(plan[1])
#         l = time.time()
#         print('f', l-t)
#    # planner.execute_plan(robot_trajectory)
#     #print('s',time.time()-l)
    

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    done = controller.execute_path(
            robot_trajectory, 
            rate=100, timeout=3)
    

    right_gripper = robot_gripper.Gripper('right_gripper')
    if close == 'open':
        right_gripper.open()
    elif close == 'close':
        right_gripper.close()
    # rospy.sleep(.05)
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    # try:
    #     trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(1.0))
    # except Exception as e:
    #     print(e)

    # current_pos = tuple([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])


def pid(limb, kin):
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
        return controller

def move(request, compute_ik, x, y, z, close):
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
    # print(response)
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

    save_path = 'pictures/logitech_output/image.jpg'

    number_of_blocks = 4 # TODO: FIX THIS!
    temp_ar_tag_index = number_of_blocks

    sorting_algorithm = insertion_sort

    ar_tags_to_sort = [i for i in range(number_of_blocks)]
    rospy.wait_for_service('compute_ik')

    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        right_gripper = robot_gripper.Gripper('right_gripper')
        right_gripper.open()
        input('Starting sort - Press [ Enter ]: ')
        pos_list = [] # list of ar tag coordinates (x, y, z)
        

        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        link = "right_gripper_tip"
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = "base"

        def scan_ar_tags(pos_list):
            for tag in ar_tags_to_sort:
                try:
                    trans = tfBuffer.lookup_transform("base", f"ar_marker_{tag}", rospy.Time(0), rospy.Duration(1.0))
                    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
                    pos_list.append(tuple(tag_pos))
                    # position_to_index[tuple(tag_pos)] = tag
                except Exception as e:
                    print(e)
            return pos_list
        
        pos_list = scan_ar_tags(pos_list)

        assert(len(pos_list) > 0) 

        # if len(position_to_index.values()) != number_of_blocks:
        #     # move to a found ar tag -> sweep!
        #     print("Sweeping since missing ar tags")
        #     sweep(request, compute_ik, pos_list[0][0], pos_list[0][1], 0) # move to an arbitrary ar tag TODO: maybe don't have it be arbitrary
        #     epsilon = 0.0
        #     while len(position_to_index.values()) != number_of_blocks: 
        #         epsilon += 0.1

        #         input(f'Sweep left/right by {epsilon} - Press [ Enter ]')

        #         print("Performing sweep in direction 1")
        #         sweep(request, compute_ik, pos_list[0][0], pos_list[0][1] + epsilon, 0.3)
        #         pos_list, position_to_index = scan_ar_tags(pos_list, position_to_index)

        #         print("Performing sweep in direction 2")
        #         sweep(request, compute_ik, pos_list[0][0], pos_list[0][1] - epsilon, 0.3)
        #         pos_list, position_to_index = scan_ar_tags(pos_list, position_to_index)

        # print('unsorted pos_list')
        # for pos in pos_list: print(pos_dict[pos])
        
        pos_list.sort(key=lambda triple: triple[1]) # sort by y-value instead of x-value since that corresponds to location in base_frame
        index_to_position = {i:pos_list[i] for i in range(len(pos_list))} # dictionary of position (x,y,z) -> ar tag index (currently 0 -> 3)

        # print("sorted position list is ")
        # for pos in pos_list: print(pos_dict[pos])
        # print("position dict is ", pos_dict)

        initial_value_order = []
       
        # for pos in pos_list:
        #     index_to_position[len(initial_value_order)] = pos
        #     initial_value_order.append(position_to_index[pos])
        # print(initial_value_order)

        # color_to_ar = {initial_color_id : ar_id for initial_color_id, ar_id in zip(color_order, initial_value_order)}
        # color_to_ar[temp_ar_tag_index] = temp_ar_tag_index

        # TODO: this should be done before/during sweep, no reason to do it after
        if sorting_algorithm != merge_sort:
            trans = tfBuffer.lookup_transform("base", f"ar_marker_{temp_ar_tag_index}", rospy.Time(0), rospy.Duration(5.0))
            temp_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
            index_to_position[temp_ar_tag_index] = tuple(temp_pos)

        # print("color_to_ar", color_to_ar)
        # print("initial value order (ar tag)", initial_value_order)

        input('Take Picture? Press [ Enter ]')

        capture_image(save_path)
        color_order = get_color_order(save_path)

        print("COLOR_ORDER:", color_order)
        
        if sorting_algorithm == merge_sort:
            print("going into merge sort!!!")
            movements =[]
            color_order2 = [(color_order[i], i) for i in range(len(color_order))]
            print("Original array:", color_order)
            sorted_array = merge_sort(color_order2, movements)
            print("Sorted array:", sorted_array)
            # print(movements)
            sorted_movements = sorted(movements, key=lambda x: x.level)
            temp = set([i for i in range(len(color_order))])
            for movement in sorted_movements:
                if movement.level != 0:
                    break
                if movement.start in temp:
                    temp.remove(movement.start)
            for num in temp:
                sorted_movements.append(Entry(num,num, 0))    
            sorted_movements = sorted(sorted_movements, key=lambda x: (x.level, x.end))

            print(sorted_movements)
            input()
            level_counter = 0
            flag = False
            for entry in sorted_movements:
                try:
                    offset = .1
                    print(offset)
                    if level_counter == 4:
                        for i in range(len(index_to_position)):
                            index_to_position[i] = (index_to_position[i][0]-offset, index_to_position[i][1], index_to_position[i][2])
                        level_counter = 0
                    print("entry is ", entry)
                    print('Move to next block - Press [ Enter ]')
                    print(f'picking up from {entry.start}')
                    print("index to position dict is ", index_to_position)

                    # move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], 0, 2)
                    # move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], -.15, 1) # 1 means close
                    # move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], 0, 2)
                    print('go above', index_to_position[entry.start])
                    # index_to_position[color_to_ar[entry.start]][0]
                    linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], .2)
                    if not flag:
                        linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], .15)
                        flag = True
                    # input('after 1st move')
                    print('go down', index_to_position[entry.start])
                    linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], -.025, 'close')
                    print('go back up', index_to_position[entry.start])
                    linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], .2)

                    print(f'placing at {entry.end}')
                    # move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], 0, 2)
                    # move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], -.15, 0) # 0 means open
                    # move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], 0, 2)
                    print("4th move", index_to_position[entry.end])
                    linear_trajectory_move(entry.end, index_to_position[entry.end][0] - offset, index_to_position[entry.end][1], 0.2)
                    print("5th move", index_to_position[entry.end])
                    linear_trajectory_move(entry.end, index_to_position[entry.end][0] - offset, index_to_position[entry.end][1], -.025, 'open')
                    print("6th move", index_to_position[entry.end])
                    linear_trajectory_move(entry.end, index_to_position[entry.end][0] - offset, index_to_position[entry.end][1], 0.2)
                    level_counter+=1
                    
                    
                    
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
        
        else:
            print("color_order", color_order)
            selection_sort_visualization = sorting_algorithm(color_order) # use color_order instead of initial_value_order
            print("list after sort ", selection_sort_visualization)

            # print("ind dictionary", index_to_position)
            input("Start moving! Press [ Enter ]")

            flag = False
            for entry in selection_sort_visualization:
                try:
                    print("entry is ", entry)
                    print('Move to next block - Press [ Enter ]')
                    print(f'picking up from {entry.start}')

                    # move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], 0, 2)
                    # move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], -.15, 1) # 1 means close
                    # move(request, compute_ik, index_to_position[entry.start][0], index_to_position[entry.start][1], 0, 2)
                    print('go above', index_to_position[entry.start])
                    # index_to_position[color_to_ar[entry.start]][0]
                    linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], .2)
                    if not flag:
                        linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], .15)
                        flag = True
                    input('after 1st move')
                    print('go down', index_to_position[entry.start])
                    linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], -.025, 'close')
                    print('go back up', index_to_position[entry.start])
                    linear_trajectory_move(entry.start, index_to_position[entry.start][0], index_to_position[entry.start][1], .2)

                    print(f'placing at {entry.end}')
                    # move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], 0, 2)
                    # move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], -.15, 0) # 0 means open
                    # move(request, compute_ik, index_to_position[entry.end][0], index_to_position[entry.end][1], 0, 2)
                    print("4th move", index_to_position[entry.end])
                    linear_trajectory_move(entry.end, index_to_position[entry.end][0], index_to_position[entry.end][1], 0.2)
                    print("5th move", index_to_position[entry.end])
                    linear_trajectory_move(entry.end, index_to_position[entry.end][0], index_to_position[entry.end][1], -.025, 'open')
                    print("6th move", index_to_position[entry.end])
                    linear_trajectory_move(entry.end, index_to_position[entry.end][0], index_to_position[entry.end][1], 0.2)
                    
                    
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()

