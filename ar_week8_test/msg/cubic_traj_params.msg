# This is the cubic_traj_params message file.
# It is used to create a message of special type containing the randomly genereated parameters
# In this case the points_generator node is sending this message type to the "params" topic, to which the cubic_traj_planner node 
# is subscribed thus it will receive the message containing the random paramters(e,g p0, v0, ... etc)

float32 p0
float32 pf
float32 v0
float32 vf
float32 t0
float32 tf