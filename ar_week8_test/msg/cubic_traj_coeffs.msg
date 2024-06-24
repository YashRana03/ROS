# This is the cubic_traj_coeffs message file.
# It is used to create a message of special type containing the calculated trajectory coefficients and time values(t0, tf)
# In this case the cubic_traj_planner node is sending this message type to the "coeffs" topic, to which the plot_cubic_traj node is 
# subscribed thus it will receive the message containing the trajectory coefficients (e.g a0, a1, ... t0, tf)

float32 a0
float32 a1
float32 a2
float32 a3
float32 t0
float32 tf
