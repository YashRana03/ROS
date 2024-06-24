#!/usr/bin/env python

# Importing the required libraries
from ar_week8_test.srv import compute_cubic_traj,compute_cubic_trajResponse
import rospy
import numpy as np

# This function computes the coefficients from the request data and returns it to the requesting node
def compute_coeffs(req):

    # declaring the matrices required for the computation of the coefficients from p0, v0, pf, vf
    # the polynomial equation being solved for is: pos(t) = a0 + a1*t + a2*t**2 + a3*t**3

    # M*a = c therefore a = M^-1*c
    m_matrix = np.array([
        [1, req.t0, (req.t0)**2, (req.t0)**3], 
        [0, 1, 2*req.t0, 3*(req.t0)**2], 
        [1, req.tf, (req.tf)**2, (req.tf)**3], 
        [0, 1, 2*req.tf, 3*(req.tf)**2]
    ])

    c_matrix = np.array([
        req.p0, 
        req.v0, 
        req.pf, 
        req.vf
    ])

    a_matrix = np.linalg.solve(m_matrix, c_matrix) # This is a numpy method that allows to solve the equation easily

    # the computed coefficients are returned using an instance of compute_cubic_trajResponse (the response of the service)
    return compute_cubic_trajResponse(a_matrix[0], a_matrix[1], a_matrix[2], a_matrix[3])


def compute_coeffs_server():

    rospy.init_node('compute_coeffs_server') # Initialising the "compute_coeffs_server" node
    # Line below creates a service called "compute_coeffs" which receives requests of type "compute_cubic_traj" and calls the handler "compute_coeffs"
    # with the received request
    s = rospy.Service('compute_coeffs', compute_cubic_traj, compute_coeffs) 
    rospy.spin()

# This checks if the file is being executed as a the main file and not by another file
if __name__ == "__main__":
    # calls the compute_coeffs_server() function 
    compute_coeffs_server()