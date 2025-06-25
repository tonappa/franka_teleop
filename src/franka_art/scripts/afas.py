#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
import mediapipe as mp
import numpy as np
import os

# Inizializza MediaPipe Gesture Recognizer
# Assicurati che il percorso del modello sia corretto
model_path = os.path.join(os.path.dirname(__file__), '/home/tonappa/Desktop/franka_teleop/src/franka_art/models/gesture_recognizer.task')
try:
    base_options = mp.tasks.BaseOptions(model_asset_path=model_path)
    options = mp.tasks.vision.GestureRecognizerOptions(base_options=base_options)
    recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
    rospy.loginfo("MediaPipe Gesture Recognizer caricato con successo.")
except Exception as e:
    rospy.logerr(f"Errore nel caricamento del modello MediaPipe: {e}")
    rospy.signal_shutdown("Impossibile caricare il modello di riconoscimento gesti.")


bridge = CvBridge()

def image_callback(msg):
    try:
        # Converti l'immagine ROS in un'immagine OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    # Converti l'immagine OpenCV in un formato compatibile con MediaPipe
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv_image)

    # Esegui il riconoscimento dei gesti
    recognition_result = recognizer.recognize(mp_image)

    if recognition_result.gestures:
        for gesture_list in recognition_result.gestures:
            for gesture in gesture_list:
                gesture_name = gesture.category_name
                score = gesture.score
                rospy.loginfo(f"Gesto riconosciuto: {gesture_name} (Score: {score:.2f})")
    else:
        rospy.loginfo("Nessun gesto riconosciuto.")

    # Puoi anche visualizzare l'immagine con le annotazioni se vuoi
    # (richiede un display X server)
    # cv2.imshow("RealSense Feed", cv_image)
    # cv2.waitKey(1)

def gesture_recognition_node():
    rospy.init_node('gesture_recognizer_node', anonymous=True)

    # Sottoscriviti al topic dell'immagine dal RealSense
    # Controlla il nome del topic esatto del tuo RealSense (spesso /camera/color/image_raw o /camera/rgb/image_raw)
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.loginfo(f"Nodo di riconoscimento gesti avviato, sottoscritto a {image_topic}")

    rospy.spin()

    # Al termine, chiudi la finestra OpenCV se aperta
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        gesture_recognition_node()
    except rospy.ROSInterruptException:
        pass