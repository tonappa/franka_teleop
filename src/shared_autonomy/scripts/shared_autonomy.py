#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float32
import numpy as np
import yaml
import os
import matplotlib

OPT_SOL = False  # Set to True to use the optimal solution function


######################################################################################################
# Function to blend the autonomous and teleoperated poses with the blending factor
def ex_blending_function(a, b, c):

    blended_pose = PoseStamped()

    ############################################################
    #   Write here your code!                                  #
    #   Write the blending function logic here assuming the    #
    #   knowledge of the teleoperation pose (a), autonomy pose #
    #   (b), and the blending factor (c)                       #
    ############################################################
    #                                                          #



    #                                                          #
    ############################################################
    
    blended_pose.pose.orientation = blending_function_orientation(teleop_pose, autonomy_pose, blending_factor).pose.orientation

    return blended_pose
##############################################################################################################
# Function to create the linear function 
def ex_optimal_solution(indipendent_variable, param_vec):

    ############################################################
    #   Write here your code!                                  #
    #   Write here your optimal solution with the blending     #
    #   function, varying the blending factor through a chosen #
    #   independent variable                                   #
    ############################################################
    #                                                          #



    #                                                          #
    ############################################################


    return blending_factor
###############################################################################################################




























##############################################################################################################
# Function to blend the autonomous and teleoperated poses with the blending factor
def blending_function(teleop_pose, autonomy_pose, blending_factor):#

    blended_pose = PoseStamped()

    # Blending the positions and orientations
    blended_pose.pose.position.x    = (1 - blending_factor) * teleop_pose.pose.position.x + blending_factor * autonomy_pose.pose.position.x
    blended_pose.pose.position.y    = (1 - blending_factor) * teleop_pose.pose.position.y + blending_factor * autonomy_pose.pose.position.y
    blended_pose.pose.position.z    = (1 - blending_factor) * teleop_pose.pose.position.z + blending_factor * autonomy_pose.pose.position.z


    blended_pose.pose.orientation = blending_function_orientation(teleop_pose, autonomy_pose, blending_factor).pose.orientation


    return blended_pose
###############################################################################################################

##############################################################################################################
# Function to create the linear function 
def optimal_solution(actual_franka_position, final_goal_position, param_vec):

    min_dist  = param_vec[0]
    max_dist  = param_vec[1]
    min_blend = param_vec[2]
    max_blend = param_vec[3]

    c = (max_blend - min_blend) / (max_dist - min_dist)
    print(f"c: {c}")

    distance = np.linalg.norm(np.array(final_goal_position) - np.array(actual_franka_position))


    # Calculate the optimal solution based on the distance and the blending factor
    if distance < min_dist:
        blending_factor = max_blend
    elif distance > max_dist:
        blending_factor = min_blend
    else:
        # Linear function: blending_factor increases from 0.2 to 0.95 as distance goes from 0.2 to a chosen max (e.g., 1.0)
        # Clamp distance to [min_dist, max_dist]
        # Linear interpolation
        blending_factor = c * (distance - min_dist) + min_blend 

    return blending_factor
###############################################################################################################






# Inizializza le variabili globali
teleop_pose   = None
autonomy_pose = None
blending_param = rospy.get_param('/shared_autonomy/initial_blending_param', 0.0) # Fattore di blending iniziale



# Callback to update the teleoperation pose
def teleop_pose_callback(msg):
    global teleop_pose
    teleop_pose = msg
    # print(f"Teleoperation pose updated: {teleop_pose.pose.position.x}, {teleop_pose.pose.position.y}, {teleop_pose.pose.position.z}")


# Callback to update the autonomy pose
def autonomy_pose_callback(msg):
    global autonomy_pose
    autonomy_pose = msg


# Callback to read the blending parameter
def blending_param_callback(msg):
    global blending_param
    blending_param = msg.data



def blending_function_orientation(teleop_pose, autonomy_pose, blending_factor):

    blended_pose = PoseStamped()
    blended_pose.pose.orientation.w = (1 - blending_factor) * teleop_pose.pose.orientation.w + blending_factor * autonomy_pose.pose.orientation.w
    blended_pose.pose.orientation.x = (1 - blending_factor) * teleop_pose.pose.orientation.x + blending_factor * autonomy_pose.pose.orientation.x
    blended_pose.pose.orientation.y = (1 - blending_factor) * teleop_pose.pose.orientation.y + blending_factor * autonomy_pose.pose.orientation.y
    blended_pose.pose.orientation.z = (1 - blending_factor) * teleop_pose.pose.orientation.z + blending_factor * autonomy_pose.pose.orientation.z

    # Normalizza l'orientamento
    norm = (blended_pose.pose.orientation.w ** 2 + blended_pose.pose.orientation.x ** 2 +
            blended_pose.pose.orientation.y ** 2 + blended_pose.pose.orientation.z ** 2) ** 0.5
    blended_pose.pose.orientation.w /= norm
    blended_pose.pose.orientation.x /= norm
    blended_pose.pose.orientation.y /= norm
    blended_pose.pose.orientation.z /= norm

    return blended_pose





def main():

    rospy.init_node('shared_autonomy_node')

    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)

    # Subscriber ai topic di teleoperazione e autonomia
    rospy.Subscriber('/teleop_pose', PoseStamped, teleop_pose_callback)
    rospy.Subscriber('/autonomy_pose', PoseStamped, autonomy_pose_callback)
    rospy.Subscriber('/blending_param', Float32, blending_param_callback)

    rate = rospy.Rate(30)  # 30 Hz

    package_dir = os.path.dirname(os.path.abspath(__file__))  # cartella scripts
    config_path = os.path.join(package_dir, "..", "config", "param_blending_function.yaml")

    # Leggi il file YAML
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)

    # Estrai i parametri
    params = config["blending_function_params"]

    # Crea il vettore
    params_vec = [params["min_dist"], params["max_dist"], params["min_blend"], params["max_blend"]]

    

    while not rospy.is_shutdown():

        # Costruisci un messaggio PoseStamped 
        blended_pose = PoseStamped()
        blended_pose.header.stamp = rospy.Time.now()
        blended_pose.header.frame_id = "panda_link0"
 
        # I valori di teleop_pose e autonomy_pose vengono aggiornati tramite subscriber
        global teleop_pose, autonomy_pose, blending_param
    
        
        # autonomy_pose = teleop_pose

        # Assicurati che entrambi i messaggi siano stati ricevuti prima di procedere
        if teleop_pose is None or autonomy_pose is None:
            rospy.logwarn("Waiting for both teleop_pose and autonomy_pose...")
            rate.sleep()
            continue

        if OPT_SOL == True:
            blending_param = optimal_solution(
                np.array([teleop_pose.pose.position.x, teleop_pose.pose.position.y, teleop_pose.pose.position.z]),
                np.array([autonomy_pose.pose.position.x, autonomy_pose.pose.position.y, autonomy_pose.pose.position.z]),
                params_vec
            )

        print(f"Blending parameter: {blending_param:.2f}")
        # Blending function application
        blended_pose = blending_function(teleop_pose, autonomy_pose, blending_param)
        # print(f"Blending parameter: {blending_param:.2f}")
        # print(f"Teleop Pose: {teleop_pose.pose.position.x}, {teleop_pose.pose.position.y}, {teleop_pose.pose.position.z}")
        # print(f"Autonomy Pose: {autonomy_pose.pose.position.x}, {autonomy_pose.pose.position.y}, {autonomy_pose.pose.position.z}")
        # print(f"Blended Pose: {blended_pose.pose.position.x}, {blended_pose.pose.position.y}, {blended_pose.pose.position.z}")

        # Publish the PoseStamped message
        blended_pose.header.stamp = rospy.Time.now()
        # rospy.loginfo("Publishing target pose...")
        pub.publish(blended_pose)

        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

