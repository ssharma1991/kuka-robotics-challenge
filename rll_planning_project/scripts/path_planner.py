#! /usr/bin/env python

import rospy
import actionlib
from rll_planning_project.srv import *
from rll_planning_project.msg import *
from geometry_msgs.msg import Pose2D
from heapq import heappush, heappop # for priority queue
import math
import numpy as np

def plan_to_goal(req):
    """ Plan a path from Start to Goal """
    pose_start = Pose2D()
    pose_goal = Pose2D()
    pose_check_start = Pose2D()
    pose_check_goal = Pose2D()
    pose_move = Pose2D()

    rospy.loginfo("Got a planning request")

    pose_start = req.start
    pose_goal = req.goal

    move_srv = rospy.ServiceProxy('move', Move)
    check_srv = rospy.ServiceProxy('check_path', CheckPath, persistent=True)

    # Some start->goal pairs the code was tested with 
    # (0.38, -0.5, 0) -> (0.38, 0.5, 0) 
    # (0.38, 0.5, 0) -> (0.3, 0.44, 1.57)
    # (0.38, 0.5, 0) -> (-0.38, -0.5, 0) 

    # Input: map dimensions, start pose, and goal pose
    # retrieving input values  
    map_width = rospy.get_param('~map_width')
    map_length = rospy.get_param('~map_length')
    xStart, yStart, tStart = pose_start.x, pose_start.y, pose_start.theta
    xGoal, yGoal, tGoal = pose_goal.x, pose_goal.y, pose_goal.theta
    # printing input values
    rospy.loginfo("map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length)
    rospy.loginfo("start pose: x %f, y %f, theta %f", xStart, yStart, tStart)
    rospy.loginfo("goal pose: x %f, y %f, theta %f", xGoal, yGoal, tGoal)

    # Output: movement commands
    pose_check_start.x, pose_check_start.y, pose_check_start.theta= xStart, yStart, tStart
    pose_check_goal.x, pose_check_goal.y, pose_check_goal.theta= xGoal, yGoal, tGoal
    resp = check_srv(pose_check_start, pose_check_goal) # checking if the arm can move to the goal pose
    if resp.valid:
        rospy.loginfo("Valid pose")
        pose_move.x, pose_move.y, pose_move.theta = xGoal, yGoal, tGoal 
        # executing a move command towards the goal pose
        # resp = move_srv(pose_move)
    else:
        rospy.loginfo("Invalid pose")

    ###############################################
    # Implement your path planning algorithm here #
    ###############################################

    path = []

    # example motions for the gripper
    min_distance = .02 # meters
    motions = [
        # movement by min_distance in positive or negative x-direction
        [min_distance, 0, 0], [-min_distance, 0, 0],
        # movement by min_distance in positive or negative y-direction
        [0, min_distance, 0], [0, -min_distance, 0],
        # rotation on the spot by 90 deg, clockwise or counterclockwise
        [0, 0, 1.57], [0, 0, -1.57],
        # rotation by 90 deg and movement into y or x direction (grinding curves)
        [0, min_distance, 1.57], [0, min_distance, -1.57],
        [0, -min_distance, 1.57], [0, -min_distance, -1.57],
        [min_distance, 0, 1.57], [min_distance, 0, -1.57],
        [-min_distance, 0, 1.57], [-min_distance, 0, -1.57]]
    

    # A* algorithm
    grid_dim = (int(map_width*100), int(map_length*100), 4) # Based on a 1cm x 1cm grid cell
    open = []
    closed = np.zeros(grid_dim)
    g_score = np.full(grid_dim, np.inf) # cheapest exact path cost calculated to a node
    f_score = np.full(grid_dim, np.inf) # f = g(exact cost) + h(estimated to goal)
    came_from_node = np.full((int(map_width*100), int(map_length*100), 4, 3), -1)

    def normalize_angle (theta):
        # normalize angle to range [0, 2pi) radians
        if theta < 0 :
            return normalize_angle(theta + 2 * 3.14)
        elif theta >= (2 * 3.14):
            return normalize_angle(theta - 2 * 3.14)
        else:
            return theta

    def gcs2mcs(pose_gcs):
        # ground coordinates to gridmap coordinates
        x, y, theta = pose_gcs
        x_grid = (x + map_width / 2) * 100
        y_grid = (y + map_length / 2) * 100
        theta_grid = (normalize_angle(theta) / np.pi) * 2
        return (int(round(x_grid)), int(round(y_grid)), int(round(theta_grid)))

    def mcs2gcs(pose_mcs):
        # gridmap coordinates to ground coordinates
        x_grid, y_grid, theta_grid = pose_mcs
        x = (x_grid / 100.0) - (map_width / 2)
        y = (y_grid / 100.0) - (map_length / 2)
        theta = (theta_grid / 2.0) * np.pi
        return (x, y, theta)

    def dist(pose1, pose2):
        # Weighted p2 norm between two poses
        dist = np.linalg.norm(np.subtract(pose1, pose2))
        #print("Distance calculation: ", pose1, pose2, dist)
        return dist
    
    def is_move_valid(pose1_gcs, pose2_gcs):
        # Checks if moveing robotic arm from pose1 to pose2 is valid
        pose2_mcs = gcs2mcs(pose2_gcs)
        #print("")
        #print(pose2_mcs)
        if pose2_mcs[0]<0 or pose2_mcs[1]<0 or pose2_mcs[2]<0:
            return False
        if pose2_mcs[0]>=closed.shape[0] or pose2_mcs[1]>=closed.shape[1] or pose2_mcs[2]>=closed.shape[2]:
            return False
        if closed[pose2_mcs]:
            return False
        
        pose1, pose2 = Pose2D(*pose1_gcs), Pose2D(*pose2_gcs)
        res = check_srv(pose1, pose2)
        if not res.valid:
            return False
        return True    

    start_mcs = gcs2mcs((pose_start.x, pose_start.y, pose_start.theta))
    goal_mcs = gcs2mcs((pose_goal.x, pose_goal.y, pose_goal.theta))
    open.append(start_mcs)
    g_score[start_mcs] = 0
    f_score[start_mcs] = g_score[start_mcs] + dist(start_mcs, goal_mcs)
    path_found = False

    while open:
        # find node with smallest f-value
        smallest_index = None
        smallest_f_value = np.inf
        for i, node in enumerate(open):
            print(node, g_score[node], f_score[node])
            if smallest_index is None:
                smallest_index = i
                smallest_f_value = f_score[node]
            elif f_score[node] < smallest_f_value:
                smallest_index = i
                smallest_f_value = f_score[node]
        current_mcs = open.pop(smallest_index)
        closed[current_mcs] = 1
        print("")
        print("CURRENT: ", current_mcs)
        print("OPEN: ", open)
        print("GOAL: ", goal_mcs)

        if current_mcs == goal_mcs:
            path_found = True
            break
        
        current_gcs = mcs2gcs(current_mcs)
        for motion in motions:
            neighbour_gcs = tuple(np.add(current_gcs, motion))
            if is_move_valid(current_gcs, neighbour_gcs):
                neighbour_mcs = gcs2mcs(neighbour_gcs)
                if neighbour_mcs not in open:
                    # Found a new neighbouring node
                    print("NEW NODE EXPLORED")
                    open.append(neighbour_mcs)
                    g_score[neighbour_mcs] = g_score[current_mcs] + 1
                    f_score[neighbour_mcs] = g_score[neighbour_mcs] + dist(neighbour_mcs, goal_mcs)
                    came_from_node[neighbour_mcs] = current_mcs
                elif g_score[neighbour_mcs] > g_score[current_mcs] + 1:
                    # Found a better for existing open node
                    print("BETTER PATH FOUND TO OPEN NODE")
                    g_score[neighbour_mcs] = g_score[current_mcs] + 1
                    f_score[neighbour_mcs] = g_score[neighbour_mcs] + dist(neighbour_mcs, goal_mcs)
                    came_from_node[neighbour_mcs] = current_mcs
    
    # Recover the shortest path
    print("PATH FOUND = ", path_found)
    current_mcs = goal_mcs
    path = []
    while (not (current_mcs == start_mcs) and path_found):
        new_pose = Pose2D(*mcs2gcs(current_mcs))
        path.append(new_pose)
        current_mcs = tuple(came_from_node[current_mcs])
    path.reverse()
    # print("\nPath Found")
    # for pt in path:
    #     print(pt.x, pt.y, pt.theta)

    # Postprocessing path to combine straight-line small moves
    path_processed = [path[0]]
    for i in range(1,len(path)-1):
        dx = abs(path[i-1].x - path[i+1].x)
        dy = abs(path[i-1].y - path[i+1].y)
        dt = abs(path[i-1].theta - path[i+1].theta)
        if dt < pow(10,-5) and (dx < pow(10,-5) or dy < pow(10,-5)):
            continue
        path_processed.append(path[i])
    path_processed.append(path[-1])
    print("\nPath Processed")
    for pt in path_processed:
        print(pt.x, pt.y, pt.theta)

    # Request robot arm to follow computed path
    if path_processed:
        rospy.loginfo("A path was found, now trying to execute it")
        for point in path_processed:
            move_srv(point)
        return True
    rospy.loginfo("No path to goal found")
    return False


class PathPlanner:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("plan_to_goal", PlanToGoalAction, self.execute, False)
        self.server.start()

    def execute(self, req):
        plan_to_goal(req)
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('path_planner')

    server = PathPlanner()

    rospy.spin()
