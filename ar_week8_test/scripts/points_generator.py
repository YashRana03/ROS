#!/usr/bin/env python

# Importing the required libraries
import rospy
from ar_week8_test.msg import cubic_traj_params
import random

# Line below creates the publisher which will allow to publish to the "params" topic messages of type "cubic_traj_params"
pub = rospy.Publisher('params', cubic_traj_params, queue_size=10) 

# This function creates the node which produces the random values
def generateRandomVals():

    
    rospy.init_node('generator', anonymous=True) # Initialising the "generator" node
    
    data = cubic_traj_params() # Creates an object of type "cubic_traj_params"
    
    rate = rospy.Rate(0.05) # Used to make the rate of execution of the while loop 0.05Hz which is 1 execution every 20 seconds
    
    while not rospy.is_shutdown():

        # This section creates the various variables randomly between -10 and 10
        data.p0 = random.uniform(-10, 10)
        data.pf = random.uniform(-10, 10)

        data.v0 = random.uniform(-10, 10)
        data.vf = random.uniform(-10, 10)

        data.t0 = 0
        data.tf = data.t0 + random.uniform(5, 10)
        
        #rospy.loginfo(data) used for debugging
        
        pub.publish(data) # publishes the random data to the "params" topic
        rate.sleep() # makes sure that the loop waits here until the 20 seconds have not passed 

# This checks if the file is being executed as a the main file and not by another file
if __name__ == '__main__':
    # calling the function to generate random values every 20 seconds forever
    generateRandomVals()