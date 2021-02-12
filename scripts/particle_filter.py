#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from likelihood_field import LikelihoodField

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random

import random


def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

#testing
def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    val = np.digitize(random_sample(n), bins)
    print(val)
    inds = values[val]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):
        print("self init\n")

        # once everything is setup initialized will be set to true
        self.initialized = False


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        #self.occupancy_field = None


        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        self.likelihood_field = LikelihoodField()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data

        #self.occupancy_field = OccupancyField(data) this is causing an error?



    def initialize_particle_cloud(self):

        # TODO
        print("initialize cloud\n")
        self.particle_cloud = []
        #print(self.map.data)
        #print(len(self.map.data))

        xrange = self.map.info.width
        yrange = self.map.info.height

        #xrange = (self.map.info.width / 2)
        #yrange = (self.map.info.height / 2)

        random.seed()

        mapspace = []

        for c in range(len(self.map.data)):
            if self.map.data[c] == 0:
                mapspace.append(c)



        for i in range(self.num_particles):
            p = Pose()
            # Position

            randompose =  random.choice(mapspace);

            p.position.x = ((randompose % xrange - xrange/2 + self.map.info.origin.position.x) * self.map.info.resolution) #random.randint(-xrange, xrange)
            p.position.y = ((randompose / yrange - yrange/2 + self.map.info.origin.position.y) * self.map.info.resolution)  #random.randint(-yrange, yrange)

            p.position.z = 0
            # Orientation / Angle
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, random.uniform(0, np.pi)) # ask
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # TODO: Do i check to see if this p already exists?
            # If it does do i do it again or do I increase the weight

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)



        self.normalize_particles()

        self.publish_particle_cloud()

        print(self.num_particles)


    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        # TODO
        total  = 0
        for p in self.particle_cloud: # add up all weights
            total += p.w

        for p in self.particle_cloud: # divide each weight by the total
            p.w = p.w / total

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # TODO
        """
        resample the particles - the probability of selecting a given particle
        for the new sample is equal to the weight of the particle
        """

        weightlist = []
        for p in self.particle_cloud:
            # print(p.w)
            weightlist.append(p.w)
        print(self.num_particles)
        # print(weightlist)
        self.particle_cloud = draw_random_sample(self.particle_cloud, weightlist, self.num_particles)

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # set to average particle
        # t stands for total, p stands for point, q for quaternion


        max_weight = 0

        tp_x = 0
        tp_y = 0
        tp_z = 0

        tq_x = 0
        tq_y = 0
        tq_z = 0
        tq_w = 0

        for p in self.particle_cloud:
            tp_x += p.pose.position.x
            tp_y += p.pose.position.y
            tp_z += p.pose.position.z
            tq_x += p.pose.orientation.x
            tq_y += p.pose.orientation.y
            tq_z += p.pose.orientation.z
            tq_w += p.pose.orientation.w

        self.robot_estimate.position.x = tp_x/self.num_particles
        self.robot_estimate.position.y = tp_y/self.num_particles
        self.robot_estimate.position.z = tp_z/self.num_particles
        self.robot_estimate.orientation.x = tq_x/self.num_particles
        self.robot_estimate.orientation.y = tq_y/self.num_particles
        self.robot_estimate.orientation.z = tq_z/self.num_particles
        self.robot_estimate.orientation.x = tq_w/self.num_particles
        # TODO



    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        # wait until initialization is complete
        if not(self.initialized):
            return

        cardinal_directions_idxs = [0, 90, 180, 270]

        for p in self.particle_cloud:
            q = 1
            for cd in cardinal_directions_idxs:

                # grab the observation at time t and laser range finder index k
                z_t_k = data.ranges[cd]

                # set the distance to the max (3.5m) if the value is greater than the max value
                # it would also be fine to skip this sensor measurement according to the
                # likelihood field algorithm formulation
                if (z_t_k > 3.5):
                    z_t_k = 3.5

                # get the orientation of the robot from the quaternion (index 2 of the Euler angle)
                theta = get_yaw_from_pose(p.pose)

                # TODO: In case the sensor is not at 0,0
                # sin_theta = math.sin(theta)
                # cos_theta = math.cos(theta)
                # translate and rotate the laser scan reading from the robot to the particle's
                # location and orientation
                x_z_t_k = p.pose.position.x + z_t_k * math.cos(theta + (cd * math.pi / 180.0))
                y_z_t_k = p.pose.position.y + z_t_k * math.sin(theta + (cd * math.pi / 180.0))

                # if x_z_t_k > 10:
                #     x_z_t_k = 10
                #
                # if x_z_t_k < -10:
                #     x_z_t_k = -10
                #
                # if y_z_t_k > 10:
                #     y_z_t_k = 10
                #
                # if y_z_t_k < -10:
                #     y_z_t_k = -10


                # find the distance to the closest obstacle
                closest_obstacle_dist = self.likelihood_field.get_closest_obstacle_distance(x_z_t_k, y_z_t_k)
                if(closest_obstacle_dist != closest_obstacle_dist):
                    closest_obstacle_dist = 10
                    print(x_z_t_k)
                    print(y_z_t_k)
                # compute the probability based on a zero-centered gaussian with sd = 0.1
                prob = compute_prob_zero_centered_gaussian(closest_obstacle_dist, 0.1)

                # multiply all sensor readings together
                q = q * prob
                    # print(q)
                    # print(prob)
                    # print(temp)

                # print everything out so we can see what we get and debug
                # print(p)
                # print("Scan[", cd, "]: ", z_t_k)
                # print("\t", cd, ": [", x_z_t_k, ", ", y_z_t_k, "]")
                # print("\tobs dist: ", closest_obstacle_dist)
                # print("\tprob: ", prob, "\n")
            p.w = q




    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        delta_x = curr_x - old_x

        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        delta_y = curr_y - old_y


        delta_move = math.sqrt((delta_x * delta_x) + (delta_y * delta_y))  # total move distance
        # TODO is this how we handle angles?
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        delta_yaw = curr_yaw - old_yaw

        #print(delta_x)
        #print(delta_y)

        for p in self.particle_cloud:
            #noise = random.random()
            theta = delta_yaw + get_yaw_from_pose(p.pose)
            p.pose.position.x += delta_move * math.cos(theta)  #+ noise
            p.pose.position.y += delta_move * math.sin(theta) #+ noise

            q = quaternion_from_euler(0.0, 0.0, theta) # ask
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]



if __name__=="__main__":


    pf = ParticleFilter()

    print("spin\n")
    rospy.spin()
