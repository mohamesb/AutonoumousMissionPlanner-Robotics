#!/usr/bin/env python3
import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan
from os import system, name
import time
import re
import fileinput
import sys
import argparse
import random
import matplotlib.animation as animation
from datetime import datetime
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point
from utils.environment import Environment_robplan
from utils.car import SimpleCar
from utils.grid import Grid_robplan
from ai_planner_modules.aiplanning import GraphPlan
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy
from utils.car import SimpleCar

from GNC import PosControl
from utils.dubins_path import DubinsPath
robot_pose = [0, 0, 0]  # Initial robot pose

def odom_callback(msg):
    global robot_pose
    q = msg.pose.pose.orientation
    (_, _, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    robot_pose = [x, y, yaw]

#2) GNC module (path-followig and PID controller for the robot)
"""  Robot GNC module ----------------------------------------------------------------------
"""
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        PI = 3.1415926535897
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():
    """
    Path-following module
    """
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # reading vehicle speed
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.trajectory = list()

        # track a sequence of waypoints
        for point in WAYPOINTS:
            self.move_to_point(point[0], point[1])
            rospy.sleep(1)
        self.stop()
        rospy.logwarn("Action done.")

        # plot trajectory
        data = np.array(self.trajectory)
        np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        plt.plot(data[:,0],data[:,1])
        plt.show()


    def move_to_point(self, x, y):
        # Here must be improved the path-following ---
        # Compute orientation for angular vel and direction vector for linear velocity

        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)

        # We should adopt different parameters for different kinds of movement
        self.pid_theta.setPID(1, 0, 0)     # P control while steering
        self.pid_theta.setPoint(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        
        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        self.pid_theta.setPID(1, 0.02, 0.2)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2

            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x = 1.5*linear   # Here can adjust speed
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()
    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            #rospy.loginfo("odom: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))



# 3) Program here your path-finding algorithm
""" Hybrid A-star pathfinding --------------------------------------------------------------------
"""

class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):

        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

class HybridAstar:
    """ Hybrid A* search procedure. """
    def __init__(self, car, grid, reverse=True, unit_theta=pi/32, dt=1e-2, check_dubins=1):
        self.car = car
        self.grid = grid
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt
        self.check_dubins = check_dubins

        self.start = self.car.start_pos
        self.goal = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)
        self.drive_steps = int(sqrt(2)*self.grid.cell_size/self.dt) + 1
        self.arc = self.drive_steps * self.dt
        self.phil = [-self.car.max_phi, 0, self.car.max_phi]
        self.ml = [1, -1]

        if reverse:
            self.comb = list(product(self.ml, self.phil))
        else:
            self.comb = list(product([1], self.phil))

        self.dubins = DubinsPath(self.car)
        self.astar = Astar(self.grid, self.goal[:2])
        
        self.w1 = 0.95 # weight for astar heuristic
        self.w2 = 0.05 # weight for simple heuristic
        self.w3 = 0.30 # weight for extra cost of steering angle change
        self.w4 = 0.20 # weight for extra cost of turning
        self.w5 = 0.50 # weight for extra cost of reversing

        self.thetas = get_discretized_thetas(self.unit_theta)
    
    def construct_node(self, pos):
        """ Create node for a pos. """

        theta = pos[2]
        pt = pos[:2]

        theta = round_theta(theta % (2*pi), self.thetas)
        
        cell_id = self.grid.to_cell_id(pt)
        #print("created node at", cell_id)
        grid_pos = cell_id + [theta]

        node = Node(grid_pos, pos)

        return node
    
    def simple_heuristic(self, pos):
        """ Heuristic by Manhattan distance. """

        return abs(self.goal[0]-pos[0]) + abs(self.goal[1]-pos[1])
        
    def astar_heuristic(self, pos):
        """ Heuristic by standard astar. """

        h1 = self.astar.search_path(pos[:2]) * self.grid.cell_size
        h2 = self.simple_heuristic(pos[:2])
        
        return self.w1*h1 + self.w2*h2

    def get_children(self, node, heu, extra):
        """ Get successors from a state. """

        children = []
        for m, phi in self.comb:

            # don't go back
            if node.m and node.phi == phi and node.m*m == -1:
                continue

            if node.m and node.m == 1 and m == -1:
                continue

            pos = node.pos
            branch = [m, pos[:2]]

            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi, m)
                branch.append(pos[:2])

            # check safety of route-----------------------
            pos1 = node.pos if m == 1 else pos
            pos2 = pos if m == 1 else node.pos
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe = self.dubins.is_turning_route_safe(pos1, pos2, d, c, r)
            # --------------------------------------------
            #print(f"Dubins path check: pos1={pos1}, pos2={pos2}, safe={safe}")
            if not safe:
                continue
            
            child = self.construct_node(pos)
            child.phi = phi
            child.m = m
            child.parent = node
            child.g = node.g + self.arc
            child.g_ = node.g_ + self.arc

            if extra:
                # extra cost for changing steering angle
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                
                # extra cost for turning
                if phi != 0:
                    child.g += self.w4 * self.arc
                
                # extra cost for reverse
                if m == -1:
                    child.g += self.w5 * self.arc

            if heu == 0:
                child.f = child.g + self.simple_heuristic(child.pos)
            if heu == 1:
                child.f = child.g + self.astar_heuristic(child.pos)
            
            children.append([child, branch])

        return children
    
    def best_final_shot(self, open_, closed_, best, cost, d_route, n=10):
        """ Search best final shot in open set. """

        open_.sort(key=lambda x: x.f, reverse=False)

        for t in range(min(n, len(open_))):
            best_ = open_[t]
            solutions_ = self.dubins.find_tangents(best_.pos, self.goal)
            d_route_, cost_, valid_ = self.dubins.best_tangent(solutions_)
        
            if valid_ and cost_ + best_.g_ < cost + best.g_:
                best = best_
                cost = cost_
                d_route = d_route_
        
        if best in open_:
            open_.remove(best)
            closed_.append(best)
        
        return best, cost, d_route
    
    def backtracking(self, node):
        """ Backtracking the path. """

        route = []
        while node.parent:
            route.append((node.pos, node.phi, node.m))
            node = node.parent
        
        return list(reversed(route))
    
    def search_path(self, heu=1, extra=False):
        """ Hybrid A* pathfinding. """

        root = self.construct_node(self.start)
        root.g = float(0)
        root.g_ = float(0)
        
        if heu == 0:
            root.f = root.g + self.simple_heuristic(root.pos)
        if heu == 1:
            root.f = root.g + self.astar_heuristic(root.pos)

        closed_ = []
        open_ = [root]

        count = 0
        while open_:
            count += 1
            best = min(open_, key=lambda x: x.f)

            open_.remove(best)
            closed_.append(best)

            # check dubins path
            if count % self.check_dubins == 0:
                solutions = self.dubins.find_tangents(best.pos, self.goal)
                d_route, cost, valid = self.dubins.best_tangent(solutions)
                
                if valid:
                    best, cost, d_route = self.best_final_shot(open_, closed_, best, cost, d_route)
                    route = self.backtracking(best) + d_route
                    path = self.car.get_path(self.start, route)
                    cost += best.g_
                    print('Shortest path: {}'.format(round(cost, 2)))
                    print('Total iteration:', count)
                    
                    return path, closed_

            children = self.get_children(best, heu, extra)

            for child, branch in children:

                if child in closed_:
                    continue

                if child not in open_:
                    best.branches.append(branch)
                    open_.append(child)

                elif child.g < open_[open_.index(child)].g:
                    best.branches.append(branch)

                    c = open_[open_.index(child)]
                    p = c.parent
                    for b in p.branches:
                        if same_point(b[-1], c.pos[:2]):
                            p.branches.remove(b)
                            break
                    
                    open_.remove(child)
                    open_.append(child)

        return None, None


def main_hybrid_a(heu,start_pos, end_pos,reverse, extra, grid_on):

    tc = map_grid_robplan()
    env = Environment_robplan(tc.obs)
    car = SimpleCar(env, start_pos, end_pos)
    grid = Grid_robplan(env)

    hastar = HybridAstar(car, grid, reverse)

    t = time.time()
    path, closed_ = hastar.search_path(heu, extra)
    print('Total time: {}s'.format(round(time.time()-t, 3)))

    if not path:
        print('No valid path!')
        return
    # a post-processing is required to have path list
    path = path[::5] + [path[-1]]
    #for i in range(len(path)):
    #    print(path[i].pos[0])
    
    branches = []
    bcolors = []
    for node in closed_:
        for b in node.branches:
            branches.append(b[1:])
            bcolors.append('y' if b[0] == 1 else 'b')

    xl, yl = [], []
    xl_np1,yl_np1=[],[]
    carl = []
    dt_s=int(25)  # samples for gazebo simulator
    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])
        if i==0 or i==len(path):
            xl_np1.append(path[i].pos[0])
            yl_np1.append(path[i].pos[1])            
        elif dt_s*i<len(path):
            xl_np1.append(path[i*dt_s].pos[0])
            yl_np1.append(path[i*dt_s].pos[1])      
    # defining way-points (traslandado el origen a (0,0))
    xl_np=np.array(xl_np1)
    xl_np=xl_np-20
    yl_np=np.array(yl_np1)
    yl_np=yl_np-11.2
    global WAYPOINTS
    WAYPOINTS=np.column_stack([xl_np,yl_np])
    #print(WAYPOINTS)
    
    start_state = car.get_car_state(car.start_pos)
    end_state = car.get_car_state(car.end_pos)

    # plot and annimation
    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
        ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid(which='both')
    else:
        ax.set_xticks([])
        ax.set_yticks([])
    
    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))
    
    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    # _branches = LineCollection(branches, color='b', alpha=0.8, linewidth=1)
    # ax.add_collection(_branches)

    # _carl = PatchCollection(carl[::20], color='m', alpha=0.1, zorder=3)
    # ax.add_collection(_carl)
    # ax.plot(xl, yl, color='whitesmoke', linewidth=2, zorder=3)
    # _car = PatchCollection(path[-1].model, match_original=True, zorder=4)
    # ax.add_collection(_car)

    _branches = LineCollection([], linewidth=1)
    ax.add_collection(_branches)

    _path, = ax.plot([], [], color='lime', linewidth=2)
    _carl = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='w', linewidth=2)
    _car = PatchCollection([])
    ax.add_collection(_car)
    
    frames = len(branches) + len(path) + 1

    def init():
        _branches.set_paths([])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])

        return _branches, _path, _carl, _path1, _car

    def animate(i):

        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']

        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _branches.set_color(bcolors)
        
        else:
            _branches.set_paths(branches)

            j = i - len(branches)

            _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])

            sub_carl = carl[:min(j+1, len(path))]
            _carl.set_paths(sub_carl[::4])
            _carl.set_edgecolor('k')
            _carl.set_facecolor('m')
            _carl.set_alpha(0.1)
            _carl.set_zorder(3)

            _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
            _path1.set_zorder(3)

            _car.set_paths(path[min(j, len(path)-1)].model)
            _car.set_edgecolor(edgecolor)
            _car.set_facecolor(facecolor)
            _car.set_zorder(3)

        return _branches, _path, _carl, _path1, _car

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=frames,
                                  interval=1, repeat=True, blit=True)

    plt.show()

# Create a map grid here for Hybrid A* 

class map_grid_robplan:
    def __init__(self):
        self.start_pos2 = [0.3, 0.3, 0]
        self.end_pos2 = [0.8, 0.5, 0]

        # [x_position, y_position, x_width, y_width]
        self.obs = [
            [3.3, 0, 2, 0.2],   # box_wall_right
            [0, 1, 0.5, 0.2],   # box_wall_left
            [1.45, 0.7, 0.5, 0.2],    # box_wp1
            [3.16, 0.7, 0.5, 0.2],   # box_wp2
            [1.2, 1.65, 0.2, 0.4],   # random_1
            [2.3, 1.65, 0.4, 0.4],   # random_2
            [3.61, 1.95, 0.4, 0.2],  # box_wp6
        ]


#4) Program here the turtlebot actions (based in your AI planner)

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite(img_title, self.image)
            return True
        else:
            return False
        
def taking_photo_exe():
    # Initialize
    camera = TakePhoto()

    # Default value is 'photo.jpg'
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    img_title = rospy.get_param('~image_title', 'photo'+dt_string+'.jpg')

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")
	#eog photo.jpg
    # Sleep to give the last log messages time to be sent

	# saving photo in a desired directory
    file_source = '/home/miguel/catkin_ws/'
    file_destination = '/home/miguel/catkin_ws/src/assigment4_ttk4192/scripts'
    g='photo'+dt_string+'.jpg'

    shutil.move(file_source + g, file_destination)
    rospy.sleep(1)

def move_robot_waypoint0_waypoint1():
    # This function executes Move Robot from 1 to 2
    # This function uses hybrid A-star
    a=0
    while a<3:
        print("Excuting Mr12")
        time.sleep(1)
        a=a+1
    print("Computing hybrid A* path")
	
    p = argparse.ArgumentParser()
    p.add_argument('-heu', type=int, default=1, help='heuristic type')
    p.add_argument('-r', action='store_true', help='allow reverse or not')
    p.add_argument('-e', action='store_true', help='add extra cost or not')
    p.add_argument('-g', action='store_true', help='show grid or not')
    args = p.parse_args()
    start_pos = [2, 2, 0]
    end_pos = [6, 6, 3*pi/4]
    main_hybrid_a(args.heu,start_pos,end_pos,args.r,args.e,args.g)
    print("Executing path following")
    turtlebot_move()


def Manipulate_OpenManipulator_x():
    print("Executing manipulate a weight")
    time.sleep(5)

def making_turn_exe():
    print("Executing Make a turn")
    time.sleep(1)
    #Starts a new node
    #rospy.init_node('turtlebot_move', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    #speed = input("Input your speed (degrees/sec):")
    #angle = input("Type your distance (degrees):")
    #clockwise = input("Clockwise?: ") #True or false

    speed = 5
    angle = 180
    clockwise = True

    #Converting from angles to radians
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0   #should be from the odometer

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()

def check_pump_picture_ir_waypoint0():
    a=0
    while a<3:
        print("Taking IR picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

def check_seals_valve_picture_eo_waypoint0():
    a=0
    while a<3:
        print("Taking EO picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

# Charging battery 
def charge_battery_waypoint0():
    print("chargin battert")
    time.sleep(5)


# Define the global varible: WAYPOINTS  Wpts=[[x_i, y_i, theta]];
global WAYPOINTS
WAYPOINTS = [[0.2,0.2, 0],[1.7,1.6, 0], [3.4, 1, pi], [3.3, 2.65, 0], [5.0, 0.3, 0], [0.9, 2.55, 0], [3.8, 1.65, 0]]

domain_file = "/home/lars/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/domain/PDDL_domain_1.pddl"
problem_file = "/home/lars/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/problem/PDDL_problem_1.pddl"

def plan_path(start_pos, goal_pos):
    env = Environment_robplan(map_grid_robplan().obs)
    car = SimpleCar(env, start_pos, goal_pos)
    grid = Grid_robplan(env)
    planner = HybridAstar(car, grid, reverse=True)

    path, _ = planner.search_path()

    if not path:
        print("❌ No valid path found from", start_pos, "to", goal_pos)
        return []

    print("✅ Path found with", len(path), "points")
    return [p[0] for p in path]  # extract (x, y, theta)



# 5) Program here the main commands of your mission planner code
""" Main code ---------------------------------------------------------------------------
"""
if __name__ == '__main__':
    try:
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print("AI planners: GraphPlan")
        print("Path-finding: Hybrid A-star")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 20.03.23")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        input_t=input("")
        rospy.init_node('mission_planner_node')
        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.sleep(1)  # Wait a moment to get the first odom message

        # 5.0) Testing the GNC module (uncomment lines to test)

        # aea
        # aea 2
        # aea 3
        # aea 4
        

        gp = GraphPlan(domain_file, problem_file)
        plan_general = gp.graphPlan()
        #pos_control = PosControl()
        # 5.3) Start mission execution 
        # convert string into functions and executing
        print("")
        print("Starting mission execution")
        # Start simulations with battery = 100%
        battery=100
        task_finished=0
        task_total=len(plan_general)
        i_ini=0
        waypoint_coords = {
        "waypoint0": [0.3, 0.3, 0],
        "waypoint1": [1.6, 0.5, 0],
        "waypoint2": [3.4, 1.1, 0],
        "waypoint3": [3.3, 2.65, -pi],
        "waypoint4": [5.0, 0.4, 0],
        "waypoint5": [0.9, 2.55, 0],
        "waypoint6": [3.8, 1.55, 0]
        }

        while i_ini < task_total:

            plan_temp=plan_general[i_ini]
            print(plan_temp)
            if plan_temp[0]=="check_pump_picture_ir":
                print("Inspect -pump")
                time.sleep(1)

            if plan_temp[0]=="check_seals_valve_picture_eo":
                print("check-valve-EO")

                time.sleep(1)
            if plan_temp[0] == "move_robot":
                start= robot_pose
                goal = waypoint_coords[plan_temp[1]]
                print("Planning path from:", start, "to:", goal)
                path = plan_path(start, goal)

                # Follow the planned path
                PosControl(path)


            if plan_temp[0]=="move_charge_robot":
                print("")
                print("Going to rechard robot")

                time.sleep(1)

            if plan_temp[0]=="charge_battery":
                print(" ")
                print("charging battery")

                time.sleep(1)


            i_ini=i_ini+1  # Next tasks


        print("")
        print("--------------------------------------")
        print("All tasks were performed successfully")
        time.sleep(10)  

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")