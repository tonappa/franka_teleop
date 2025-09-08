#!/usr/bin/env python3

from sympy import Quaternion
import rospy
import rospkg
import os
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float32
import xml.etree.ElementTree as ET
import numpy as np
from std_msgs.msg import Int32



# Extract the initial pose from the YAML file
initial_pose = rospy.get_param('/initial_pose', {
    'position': {
        'x': 0.30699088606283903,
        'y': 2.8203182887441273e-05,
        'z': 0.4870162491576909
    },
    'orientation': {
        'x': 1.0,
        'y': 0.0,
        'z': 0.0,
        'w': 0.0
    }
})

print(f"Initial pose loaded: {initial_pose}")

# Extract the position of the cylinders
# RED
pos_cylinder_red_x = rospy.get_param('~cylinder1_x', 0.4)  # X position of red cylinder
pos_cylinder_red_y = rospy.get_param('~cylinder1_y', 0.4)  # Y position of red cylinder
pos_cylinder_red_z = rospy.get_param('~cylinder1_z', 0.37) # Z position of red cylinder


# BLUE
pos_cylinder_blue_x = rospy.get_param('~cylinder2_x', 0.6)  # X position of blue cylinder
pos_cylinder_blue_y = rospy.get_param('~cylinder2_y', 0.2)  # Y position of blue cylinder
pos_cylinder_blue_z = rospy.get_param('~cylinder2_z', 0.37) # Z position of blue cylinder





# Global variable to keep track of the trajectory index
traj_number = 0
def switch_goal_callback(msg):
    global traj_number
    traj_number = msg.data

switch_goal_sub = rospy.Subscriber('/switch_goal', Int32, switch_goal_callback)


def extract_cylinder_length(sdf_path, model_name):
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for model in root.findall('.//model'):
        if model.get('name') == model_name:
            for link in model.findall('link'):
                for collision in link.findall('collision'):
                    geometry = collision.find('geometry')
                    if geometry is not None:
                        cylinder = geometry.find('cylinder')
                        if cylinder is not None:
                            length = cylinder.find('length')
                            if length is not None:
                                return float(length.text)
    rospy.logwarn(f"Model '{model_name}' not found in SDF file '{sdf_path}'")
    print(length)
    return None


def arc_trajectory(p0, pm, pf, t):
    """
    Calcola un punto lungo una traiettoria ad arco data da tre punti.
    :param p0: punto iniziale (array-like, es. [x0, y0, z0])
    :param pm: punto intermedio più alto (array-like)
    :param pf: punto finale (array-like)
    :param t: tempo normalizzato tra 0 e 1
    :return: posizione 3D in t
    """
    p0 = np.array(p0)
    pm = np.array(pm)
    pf = np.array(pf)

    position = (1 - t)**2 * p0 + 2 * (1 - t) * t * pm + t**2 * pf
    return position


def chose_trajectory(trajectory, index):
    """
    Sceglie una traiettoria in base all'indice.
    :param trajectory: lista di traiettorie
    :param index: indice della traiettoria da scegliere
    :return: traiettoria scelta
    """
    if index < len(trajectory):
        return trajectory[index]
        index += 1
    else:
        return trajectory[-1]  # mantieni l'ultimo punto se l'indice è fuori range


def main():

    rospy.init_node('robot_autonomy_node')

    pub = rospy.Publisher('/autonomy_pose', PoseStamped, queue_size=10)


    rate = rospy.Rate(100)  # 100 Hz

    # Usa rospkg per trovare il path assoluto del file .sdf
    rospack = rospkg.RosPack()
    model_path = os.path.join(
        rospack.get_path('panda_gazebo_models'),
        'models', 'red_cylinder_small', 'model.sdf'
    )

    # ATTENZIONE: devi usare il nome del modello che è dentro il tag <model name="...">
    height_cylinder = extract_cylinder_length(model_path, 'small_cylinder')
    print(f"Altezza del cilindro rosso: {height_cylinder}")

    pos_cylinder_red   = [pos_cylinder_red_x, pos_cylinder_red_y, pos_cylinder_red_z]
    pos_cylinder_blue  = [pos_cylinder_blue_x, pos_cylinder_blue_y, pos_cylinder_blue_z]

    des_orientation = [initial_pose['orientation']['x'], initial_pose['orientation']['y'], initial_pose['orientation']['z'], initial_pose['orientation']['w']]

    # Calcola i punti lungo la traiettoria ad arco

    ####### Trajectory 1 PICK1

    initial_point1 = [initial_pose['position']['x'], initial_pose['position']['y'], initial_pose['position']['z']]
    final_point1 = [pos_cylinder_red[0], pos_cylinder_red[1], pos_cylinder_red[2] + height_cylinder - 0.02]
    eps = 0.45  # Offset per evitare collisioni con il cilindro rosso
    median_point1 = [(initial_point1[0] + final_point1[0]) / 2,  final_point1[1] - 0.05, final_point1[2] + eps]

    trajectory1 = [arc_trajectory(initial_point1, median_point1, final_point1, t)
                   for t in np.linspace(0, 1, 2000)]  # 5000

    ####### Trajectory 2  POSE1

    initial_point2 = final_point1
    initial_point2[2] += 0.08  # Alza il punto iniziale per evitare collisioni
    final_point2 = [0.4, -0.4, pos_cylinder_red[2] + height_cylinder - 0.02]
    median_point2 = [(initial_point2[0] + final_point2[0]) / 2,  final_point2[1] - 0.05, final_point2[2] + eps]

    trajectory2 = [arc_trajectory(initial_point2, median_point2, final_point2, t)
                   for t in np.linspace(0, 1, 2000)]  # 5000

    ####### Trajectory 3 PICK2

    initial_point3 = final_point2
    initial_point3[2] += 0.05  # Alza il punto iniziale per evitare collisioni
    final_point3 = [pos_cylinder_blue[0], pos_cylinder_blue[1], pos_cylinder_blue[2] + height_cylinder - 0.02]
    median_point3 = [(initial_point3[0] + final_point3[0]) / 2,  final_point3[1] - 0.05, final_point3[2] + eps]

    trajectory3 = [arc_trajectory(initial_point3, median_point3, final_point3, t)
                   for t in np.linspace(0, 1, 2000)]  # 5000
    
     ####### Trajectory 4 POSE2

    initial_point4 = final_point3
    initial_point4[2] += 0.05  # Alza il punto iniziale per evitare collisioni
    final_point4 = [0.6, -0.2, pos_cylinder_red[2] + height_cylinder - 0.02]
    median_point4 = [(initial_point4[0] + final_point4[0]) / 2,  final_point4[1] - 0.05, final_point4[2] + eps]


    trajectory4 = [arc_trajectory(initial_point4, median_point4, final_point4, t)
                   for t in np.linspace(0, 1, 2000)]  # 5000
    
    ####### Trajectory 5 - HOME

    initial_point5 = final_point4
    initial_point5[2] += 0.05  # Alza il punto iniziale per evitare collisioni
    final_point5 = [initial_pose['position']['x'], initial_pose['position']['y'], initial_pose['position']['z']] 

    median_point5 = [(initial_point5[0] + final_point5[0]) / 2,  final_point5[1] - 0.05, final_point5[2] + eps]


    trajectory5 = [arc_trajectory(initial_point5, median_point5, final_point5, t)
                   for t in np.linspace(0, 1, 1000)]  # 1000

    # Dizionario di traiettorie 
    trajectory = [trajectory1, trajectory2, trajectory3, trajectory4, trajectory5]
    


    ####### Trajectory - HOME

    initial_point_home = [initial_pose['position']['x'], initial_pose['position']['y'], initial_pose['position']['z']]

    trajectory_home = [initial_point_home] 





    index = 0  # indice per scorrere la traiettoria selezionata

   
    while not rospy.is_shutdown():


        # counter = counter + 1
        #pos = chose_trajectory(trajectory1, index)

        # Scegli la traiettoria in base al valore di traj_number (aggiornato dal callback)
        prev_traj_number = getattr(main, "prev_traj_number", None)
        if traj_number == 0:
            selected_trajectory = trajectory[0]
        elif traj_number == 1:
            selected_trajectory = trajectory[1]
        elif traj_number == 2:
            selected_trajectory = trajectory[2]
        elif traj_number == 3:
            selected_trajectory = trajectory[3]
        elif traj_number == 4:
            print("Selected trajectory: Home")            
            selected_trajectory = trajectory[4]
            #print(selected_trajectory)

        else: # default
            selected_trajectory = trajectory_home # Home trajectory
        

        # Se il numero di traiettoria è cambiato, resetta l'indice
        if prev_traj_number != traj_number:
            index = 0
        main.prev_traj_number = traj_number

        if index < len(selected_trajectory):
            pos = selected_trajectory[index]
            index += 1
        else:
            pos = selected_trajectory[-1]  # mantieni l'ultimo punto se l'indice è fuori range

        # Costruisci un messaggio PoseStamped 
        autonomy_pose = PoseStamped()
        autonomy_pose.header.stamp = rospy.Time.now()
        autonomy_pose.header.frame_id = "panda_link0"

        autonomy_pose.pose.position = Point(pos[0], pos[1], pos[2])
        # autonomy_pose.pose.position = Point(pos_cylinder_red[0], pos_cylinder_red[1], pos_cylinder_red[2] + height_cylinder)
        # print(f"Autonomy Posistion: {autonomy_pose.pose.position.x}, {autonomy_pose.pose.position.y}, {autonomy_pose.pose.position.z}")
        autonomy_pose.pose.orientation = Quaternion(des_orientation[0], des_orientation[1], des_orientation[2], des_orientation[3])
        # print(f"Autonomy Orientation: {autonomy_pose.pose.orientation.x}, {autonomy_pose.pose.orientation.y}, {autonomy_pose.pose.orientation.z}, {autonomy_pose.pose.orientation.w}")
        
        # Publish the PoseStamped message
        autonomy_pose.header.stamp = rospy.Time.now()
        # rospy.loginfo("Publishing target pose...")
        pub.publish(autonomy_pose)

        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
