#!/usr/bin/env python3

from sympy import Quaternion
import rospy
import rospkg
import os
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float32
import xml.etree.ElementTree as ET
import numpy as np



# Extract the initial pose from the YAML file
initial_pose = rospy.get_param('/shared_autonomy/initial_pose', {
    'position': {'x': 0.3, 'y': 0.0, 'z': 0.4},
    'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
})
print(f"Initial pose loaded: {initial_pose}")

# Extract the position of the cylinders
# RED
pos_cylinder_red_x = rospy.get_param('~cylinder1_x', 0.4)  # X position of red cylinder
pos_cylinder_red_y = rospy.get_param('~cylinder1_y', 0.4)  # Y position of red cylinder
pos_cylinder_red_z = rospy.get_param('~cylinder1_z', 0.37) # Z position of red cylinder


# GREEN
pos_cylinder_green_x = rospy.get_param('~cylinder2_x', 0.6)  # X position of green cylinder
pos_cylinder_green_y = rospy.get_param('~cylinder2_y', 0.2)  # Y position of green cylinder
pos_cylinder_green_z = rospy.get_param('~cylinder2_z', 0.37) # Z position of green cylinder


# BLUE
pos_cylinder_blue_x = rospy.get_param('~cylinder3_x', 0.2)  # X position of blue cylinder
pos_cylinder_blue_y = rospy.get_param('~cylinder3_y', 0.6)  # Y position of blue cylinder
pos_cylinder_blue_z = rospy.get_param('~cylinder3_z', 0.37) # Z position of blue cylinder



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

    pos_cylinder_red = [pos_cylinder_red_x, pos_cylinder_red_y, pos_cylinder_red_z]
    pos_cylinder_green = [pos_cylinder_green_x, pos_cylinder_green_y, pos_cylinder_green_z]
    pos_cylinder_blue = [pos_cylinder_blue_x, pos_cylinder_blue_y, pos_cylinder_blue_z]

    des_orientation = [initial_pose['orientation']['x'], initial_pose['orientation']['y'], initial_pose['orientation']['z'], initial_pose['orientation']['w']]

    # Calcola i punti lungo la traiettoria ad arco
    initial_point = [initial_pose['position']['x'], initial_pose['position']['y'], initial_pose['position']['z']]
    final_point = [pos_cylinder_red[0], pos_cylinder_red[1], pos_cylinder_red[2] + height_cylinder - 0.05]
    eps = 0.45  # Offset per evitare collisioni con il cilindro rosso
    median_point = [(initial_point[0] + final_point[0]) / 2,  final_point[1] - 0.1, final_point[2] + eps]
    
    trajectory = [arc_trajectory(initial_point, median_point, final_point, t) 
              for t in np.linspace(0, 1, 5000)]
    
    print(f"Trajectory points: {trajectory}")

    # Inizializza l'indice per la traiettoria
    index = 0

    while not rospy.is_shutdown():




        if index < len(trajectory):
            print(f"Index: {index}, Total Points: {len(trajectory)}")
            pos = trajectory[index]
            print(f"Current Position: {pos}")
            index += 1
        else:
            pos = trajectory[-1]  # mantieni l'ultimo punto

        # Costruisci un messaggio PoseStamped 
        autonomy_pose = PoseStamped()
        autonomy_pose.header.stamp = rospy.Time.now()
        autonomy_pose.header.frame_id = "panda_link0"

        autonomy_pose.pose.position = Point(pos[0], pos[1], pos[2])

        # autonomy_pose.pose.position = Point(pos_cylinder_red[0], pos_cylinder_red[1], pos_cylinder_red[2] + height_cylinder)
        print(f"Autonomy Posistion: {autonomy_pose.pose.position.x}, {autonomy_pose.pose.position.y}, {autonomy_pose.pose.position.z}")
        autonomy_pose.pose.orientation = Quaternion(des_orientation[0], des_orientation[1], des_orientation[2], des_orientation[3])
        print(f"Autonomy Orientation: {autonomy_pose.pose.orientation.x}, {autonomy_pose.pose.orientation.y}, {autonomy_pose.pose.orientation.z}, {autonomy_pose.pose.orientation.w}")
        
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
