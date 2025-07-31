#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from visualization_msgs.msg import Marker
import tf
import tf2_ros
import geometry_msgs.msg

def publish_object_state():
    # Nome del modello in Gazebo (dal tuo file SDF)
    model_name = 'red_cube'  # Cambia con il nome del tuo oggetto
    
    # Frame di riferimento (il mondo di Gazebo)
    reference_frame = 'world'

    rospy.init_node('gazebo_object_publisher', anonymous=True)

    # 1. Publisher per il Marker di visualizzazione
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # 2. Broadcaster per la trasformata TF
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Aspetta che il servizio di Gazebo sia disponibile
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rospy.loginfo("Inizio a pubblicare lo stato dell'oggetto da Gazebo a RViz.")

    rate = rospy.Rate(30) # 30 Hz
    while not rospy.is_shutdown():
        try:
            # Chiama il servizio per ottenere la posa dell'oggetto
            model_state = get_model_state(model_name, reference_frame)

            if model_state.success:
                pose = model_state.pose
                
                # --- Pubblica la trasformata TF ---
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = reference_frame # Frame padre (es. 'world' o 'odom')
                t.child_frame_id = model_name      # Frame figlio (il nome dell'oggetto)
                t.transform.translation = pose.position
                t.transform.rotation = pose.orientation
                tf_broadcaster.sendTransform(t)

                # --- Pubblica il Marker per la visualizzazione ---
                marker = Marker()
                marker.header.frame_id = reference_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "gazebo_objects"
                marker.id = 0
                marker.type = Marker.CUBE # Il tuo oggetto è un box
                marker.action = Marker.ADD
                
                marker.pose = pose
                
                # Dimensioni del marker (devono corrispondere a quelle nel file SDF)
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.1
                
                # Colore del marker (Rosso, come in Gazebo)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0 # Opacità
                
                marker.lifetime = rospy.Duration()
                
                marker_pub.publish(marker)

        except rospy.ServiceException as e:
            rospy.logerr("Servizio GetModelState fallito: %s" % e)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_object_state()
    except rospy.ROSInterruptException:
        pass