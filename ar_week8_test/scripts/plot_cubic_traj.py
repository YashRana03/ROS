#!/usr/bin/env python

# Importing the required libraries
import rospy
from ar_week8_test.msg import cubic_traj_params
from ar_week8_test.msg import cubic_traj_coeffs
from std_msgs.msg import Float32

# Declaring the 3 publishers need to publish the trajectories for: pos, vel and acc
pubPOS = rospy.Publisher('trajPos', Float32, queue_size=10)
pubVEL = rospy.Publisher('trajVel', Float32, queue_size=10)
pubACC = rospy.Publisher('trajAcc', Float32, queue_size=10)


# This function calculates the trajectories for position, velocity and accelaration and publishes them to their respective topics 
def callback(data):
    # initialising variables
    valuePos = 0
    valueVel = 0
    valueAcc = 0
    counter = 0 # Counter keeps track of time

    # Following two lines compute the step size requried for the plotting at 5Hz rate (necessary to make the plotting smooth visually)
    n = data.tf*5 
    increment = (data.tf-data.t0)/n

    rate = rospy.Rate(5)  # Used to make the rate of execution of the while loop 5Hz which is 5 execution per second
    while not rospy.is_shutdown():
        # Checks if the tf time has elapsed if so stops publishing trajectory data
        if counter > data.tf:
            break
        # Following compute the trajectories for pos, vel, acc using the coefficients received
        valuePos = data.a0 + data.a1*counter + data.a2*counter**2 + data.a3*counter**3
        valueVel = data.a1 + 2*data.a2*counter + 3*data.a3*counter**2
        valueAcc = 2*data.a2 + 6*data.a3*counter
        

        # Publishing to the appropriate topics
        pubPOS.publish(valuePos)
        pubVEL.publish(valueVel)
        pubACC.publish(valueAcc)
        counter += increment # incrementing the time counter 
        rate.sleep() # makes sure that the loop waits here until the 20 seconds have not passed 





    
def plotter():

    rospy.init_node('plotter', anonymous=True) # Initialising the "plotter" node

    # Line below makes the current node listent to the "coeffs" topic where it will receive messages of type "cubic_traj_coeffs"
    # When message is received the callback function will be called with the received message as argument
    rospy.Subscriber("coeffs", cubic_traj_coeffs, callback)

    # spin() prevents python from exiting until this node is stopped
    rospy.spin()

# This checks if the file is being executed as a the main file and not by another file
if __name__ == '__main__':
    # Calling the plotter()
    plotter()