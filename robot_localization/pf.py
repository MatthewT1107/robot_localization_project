#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import rclpy
import random
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from helper_functions import draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler

class Particle(object):
    """ Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
        Attributes:
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """ Construct a new Particle
            x: the x-coordinate of the hypothesis relative to the map frame
            y: the y-coordinate of the hypothesis relative ot the map frame
            theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
            w: the particle weight (the class does not ensure that particle weights are normalized """ 
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """ A helper function to convert a particle to a geometry_msgs/Pose message """
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(position=Point(x=self.x, y=self.y, z=0.0),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    # TODO: define additional helper functions if needed

class ParticleFilter(Node):
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            last_scan_timestamp: this is used to keep track of the clock when using bags
            scan_to_process: the scan that our run_loop should process next
            occupancy_field: this helper class allows you to query the map for distance to closest obstacle
            transform_helper: this helps with various transform operations (abstracting away the tf2 module)
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            thread: this thread runs your main loop
    """
    def __init__(self):
        super().__init__('pf')
        self.base_frame = "base_footprint"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 625          # the number of particles to use

        self.d_thresh = 0.2             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update
        self.w_fast = None # fast average of particle likelihoods
        self.w_slow = None # slow average of particle likelihoods
        # TODO: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.update_initial_pose, 10)

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(ParticleCloud, "particle_cloud", qos_profile_sensor_data)

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # Get map data
        self.map_info = self.occupancy_field.map.info
        # Width and height are in terms of cells, not meters
        self.map_width = self.map_info.width
        self.map_height = self.map_info.height
        # Resolution is how many world meters are in a cell
        self.resolution = self.map_info.resolution # 0.05m/cell
        # The minimum, mid, and maximum x and y are in
        self.x_min = self.map_info.origin.position.x
        self.y_min = self.map_info.origin.position.y
        self.x_max = self.x_min + self.map_width * self.resolution
        self.y_max = self.y_min + self.map_height * self.resolution
        self.x_mid = self.x_min + (self.map_width/2) * self.resolution
        self.y_mid = self.y_min + (self.map_height/2) * self.resolution



        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)



    def pub_latest_transform(self):
        """ This function takes care of sending out the map to odom transform """
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(seconds=0.1)
        self.transform_helper.send_last_map_to_odom_transform(self.map_frame, self.odom_frame, postdated_timestamp)

    def loop_wrapper(self):
        """ This function takes care of calling the run_loop function repeatedly.
            We are using a separate thread to run the loop_wrapper to work around
            issues with single threaded executors in ROS2 """
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """ This is the main run_loop of our particle filter.  It checks to see if
            any scans are ready and to be processed and will call several helper
            functions to complete the processing.
            
            You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(self.odom_frame,
                                                                           self.base_frame,
                                                                           msg.header.stamp)
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return
        
        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(msg, self.base_frame)
        #print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        #print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()    # update based on odometry
            self.update_particles_with_laser(r, theta)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose based on particles
            self.resample_particles()               # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or \
               math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh


    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose
                (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # TODO: assign the latest pose into self.robot_pose as a geometry_msgs.Pose object
        # just to get started we will fix the robot's pose to always be at the origin

        #!!!!!!!!!!!!!!!!!!!!!!!! Currently this function does weighted means for every particle. 
        #!!!!!!!!!!!!!!!!!!!!!!!! We may want to change it to weighted means of top N particles (Victoria suggestion)

        # Make a separate list for each attribute of particle
        weights = [p.w for p in self.particle_cloud]
        x_vals = [p.x for p in self.particle_cloud]
        y_vals = [p.y for p in self.particle_cloud]
        
        # Weighted x and y means
        # zip produces (x,w) pairs for the two lists
        x_mean = sum(x*w for x,w in zip(x_vals,weights))
        y_mean = sum(y*w for y,w in zip(y_vals,weights))

        # Weighted theta mean.
        # Convert each angle into weighted horizontal and vertical components using sin and cosine
        s = sum(p.w * math.sin(p.theta) for p in self.particle_cloud)
        c = sum(p.w * math.cos(p.theta) for p in self.particle_cloud)
        # Convert components back into an angle
        theta_mean = math.atan2(s, c)
        
        # Create the guess as a particle 
        guess = Particle(x_mean,y_mean,theta_mean)

        # Send pose
        self.robot_pose = guess.as_pose()
        if hasattr(self, 'odom_pose'):
            self.transform_helper.fix_map_to_odom_transform(self.robot_pose, self.odom_pose)
        else:
            self.get_logger().warn("Can't set map->odom transform since no odom data received")
       

    def update_particles_with_odom(self):
        """ Update the particles using the newly given odometry pose.
            The function computes the value delta which is a tuple (x,y,theta)
            that indicates the change in position and angle between the odometry
            when the particles were last updated and the current odometry.

            TODO: move each particle by delta but delta is in odom fram so need to rotate into each particles own heading. 
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return
        old_theta = old_odom_xy_theta[2]
        dx, dy = delta[0], delta[1]
        dx_r = math.cos(old_theta) * dx + math.sin(old_theta) * dy
        dy_r = -math.sin(old_theta) * dx + math.cos(old_theta) * dy
        d_theta = self.transform_helper.angle_diff(new_odom_xy_theta[2], old_theta)
        for p in self.particle_cloud:
            p.x += math.cos(p.theta) * dx_r - math.sin(p.theta) * dy_r + np.random.normal(0, 0.02) #add some noise to the translation
            p.y += math.sin(p.theta) * dx_r + math.cos(p.theta) * dy_r + np.random.normal(0, 0.02) #add some noise to the translation
            p.theta = self.transform_helper.angle_normalize(p.theta + d_theta +np.random.normal(0, 0.02)) #add some noise to the rotation


    def resample_particles(self):
        """ Resample the particles according to the new particle weights.
            The weights stored with each particle should define the probability that a particular
            particle is selected in the resampling step.  You may want to make use of the given helper
            function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        # TODO: fill out the rest of the implementation
        
        length = len(self.particle_cloud)
        #n_random = 25 - testing kidnap logic
        #add more random particles if the current mathc is worse than usual (kidnapped)
        if self.w_slow:
            p_random = max(0, 1-self.w_fast/self.w_slow) #p_random is how much of the cloud needs to be replaces with random particles. w_fast/w_slow gives the ratio of how well filter is currently compared to previously. using max to cap it at 0 if it goes negative
        else:
            p_random = 0
        n_random = max(5,int(p_random * length)) #always have a min of 5 random particles
        n_keep = length - n_random

        # Get the weights of each particle in particle cloud as a list
        weights = [p.w for p in self.particle_cloud]
        
        # Use the helper function to randomly choose particles to add to new list based on weights
        # We take n_keep particles and replace n_random particles to add some randomness to the particle cloud
        new_particles = draw_random_sample(self.particle_cloud,weights,n_keep)

        
        # Update particle cloud to contain new particles
        self.particle_cloud = new_particles


        # Add random noise                                              
        for p in self.particle_cloud:                                                
            p.x += np.random.normal(0, 0.05)                                         
            p.y += np.random.normal(0, 0.05)                                         
            p.theta = self.transform_helper.angle_normalize(p.theta + np.random.normal(0, 0.05))  

        valid_particles = []
        for p in self.particle_cloud:
            d = self.occupancy_field.get_closest_obstacle_distance(p.x, p.y) #checking dist to closest obj
            if math.isfinite(d) and d > 0.01:
                valid_particles.append(p) #adding only valid particles
        n_removed = len(self.particle_cloud) - len(valid_particles) #count of particles removed
        self.particle_cloud = valid_particles #particle cloud updated with only valid particles

        # Replace each removed particle with a new random one so the count stays the same
        for _ in range(n_removed):
            self.random_particle()



        for _ in range(n_random):
            self.random_particle()
        # Reset the weights for each particle
        # May not be necessary, we'll see after testing
        for p in self.particle_cloud:
            p.w = 1.0 / len(self.particle_cloud)


        
        print(f"Particles resampled. Cloud list is {len(self.particle_cloud)}")

    def random_particle(self,xy_theta=None):
        """ Create a random, valid particle. If an initial pose is given, the particle will be generated around the pose.
            Otherwise, the particle will be completely random
            Args:
                xy_theta: A triple consisting of the mean x, y, and theta to initialize the particle cloud around.   
        """
        # While loop used as a repeat if any particle fails
        while True:
            if xy_theta is None:
                # Generate particles over the entire map
                # Completely random x,y, and theta values within map bounds
                particle_x = random.uniform(self.x_min, self.x_max)
                particle_y = random.uniform(self.y_min, self.y_max)
                particle_theta = random.uniform(0,2*math.pi)
                particle_theta = math.atan2(math.sin(particle_theta),math.cos(particle_theta))
            else:
                # Generate particles over most of the map concentrated more on the initial pose xy_theta
                # This is done using a gaussian distribution with a mean of the initial pose and a standard deviation dev for x and y
                particle_x = random.gauss(xy_theta[0], 0.5)
                particle_y = random.gauss(xy_theta[1], 0.5)
                particle_theta = random.gauss(xy_theta[2], 0.3)
                particle_theta = math.atan2(math.sin(particle_theta),math.cos(particle_theta)
)
            # Get the distance from the particle to the closest obstacle
            d = self.occupancy_field.get_closest_obstacle_distance(particle_x,particle_y)
            # If the particle is within bounds and not inside of an object, break out the loop and create particle
            if math.isfinite(d) and d > 0.01:
                break

        # Create the particle
        random_particle = Particle(particle_x,particle_y,particle_theta,1.0)
        # Add the particle to the list, repeat for each particle
        # print("Random particle made")
        self.particle_cloud.append(random_particle)
        

        
            
    def update_particles_with_laser(self, r, theta):
        """ Updates the particle weights in response to the scan data
            r: the distance readings to obstacles
            theta: the angle relative to the robot frame for each corresponding reading 
        """ 
        step = 5 #to speed it up, we are using every 5th beam
        sigma = 0.1#error of the laser map
        for p in self.particle_cloud:
            likelihood = 0.0
            for r_i, theta_i in list(zip(r, theta))[::step]:
                if r_i == 0 or not math.isfinite(r_i):
                    continue #skip invalid readings
                #where the beam would be if the robot is at this pose
                x = p.x + r_i*math.cos(p.theta + theta_i)
                y = p.y + r_i*math.sin(p.theta + theta_i)
                d = self.occupancy_field.get_closest_obstacle_distance(x, y) #dist between hit point from real obstacle on the map
                if not math.isfinite(d):
                    continue #skips if distance is invalid
                likelihood += math.exp(-d**2/(2*sigma**2)) #smaller d means closer to 1, vice versa
            p.w *= likelihood
       
        #figuring out if filter is lost (robot kidnapping)
        self.avg_likelihood = sum(p.w for p in self.particle_cloud) / len(self.particle_cloud) if self.particle_cloud else 0 #average of particle likelihoods to see how good the cloud is for the scan
        alpha_slow = 0.05 #rate of slow average. moves slower so it remembers how the particle filter behaved earlier
        alpha_fast = 0.2 #rate of fast average. it moves faster to see how the filter is performing currently
        #measuring how well the particles match the laser scan at different rates
        if self.w_slow is None:
            self.w_slow = self.avg_likelihood
            self.w_fast = self.avg_likelihood
        else:
            self.w_slow +=alpha_slow * (self.avg_likelihood - self.w_slow) #using exponential moving average so it moves the average to the new value at a certain rate (alpha is how far it moves)
            self.w_fast +=alpha_fast * (self.avg_likelihood - self.w_fast)
        self.normalize_particles()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """ Initialize the particle cloud.
           Arguments
           xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                   particle cloud around.  If this input is omitted, the odometry will be used """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose)
        self.particle_cloud = []
        # TODO create particles

        # --------------------------CREATING INITIAL PARTICLES---------------------------------------------------
        print("Initializing new cloud")
        focused_particle_num = 600 # Number of particles focused around initial pose
        random_particle_num = 25 # Number of completely random particles

        # Create the focused particles using the initial pose xy_theta
        for p in range(focused_particle_num):            
            self.random_particle(xy_theta)

        # Create 25 random particles to add some randomness to the particle cloud
        for p in range (random_particle_num):
            self.random_particle()

        self.normalize_particles()
        self.update_robot_pose()
        print(f"Particle list length:{len(self.particle_cloud)}")

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        if not self.particle_cloud: #check if list is empty
            return

        total_weight = sum(p.w for p in self.particle_cloud) #add up weights of all particles. p.w is the weight of one particle

        if total_weight <= 0: #checks if weight is zero or negative. 
            uniform_weight = 1.0 / len(self.particle_cloud) #every particle gets equal share (1 divided by no of particles)
            for p in self.particle_cloud: #iterates through each particle
                p.w = uniform_weight #assign uniform weight to each particle
            return

        for p in self.particle_cloud: 
            p.w /= total_weight #divides each particles weight by the total

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop 
        if self.scan_to_process is None:
            self.scan_to_process = msg

def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
