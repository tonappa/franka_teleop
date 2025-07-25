#!/usr-bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridgeError, CvBridge
import cv2
import mediapipe as mp
import os

# --- MODIFICA: Inizializzazione semplificata ---
bridge = CvBridge()
recognizer = None
gesture_publisher = None

# Funzione per ridimensionare mantenendo le proporzioni
def resize_image(image, width=640):
    (h, w) = image.shape[:2]
    r = width / float(w)
    dim = (width, int(h * r))
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

def init_gesture_recognizer(model_path):
    """Inizializza il riconoscitore di gesti di MediaPipe."""
    global recognizer
    try:
        if not os.path.exists(model_path):
            rospy.logerr(f"Errore: Il file del modello non esiste: {model_path}")
            rospy.signal_shutdown("Modello di riconoscimento non trovato.")
            return False

        base_options = mp.tasks.BaseOptions(model_asset_path=model_path)
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.5, # Valore di default, puoi aggiustarlo se serve
            min_tracking_confidence=0.5
        )
        recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        rospy.loginfo(f"MediaPipe Gesture Recognizer caricato da: {model_path}")
        return True
    except Exception as e:
        rospy.logerr(f"Errore nel caricamento del modello MediaPipe: {e}")
        rospy.signal_shutdown("Impossibile caricare il modello.")
        return False

def image_callback(msg):
    """
    Callback per le immagini: elabora un frame e pubblica il risultato.
    Logica semplificata: no debounce, no heartbeat.
    """
    global bridge, recognizer, gesture_publisher

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # --- MODIFICA: Ridimensiona l'immagine per performance migliori ---
        cv_image_resized = resize_image(cv_image, width=640)

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    # Converte l'immagine per MediaPipe e la processa
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv_image_resized)
    recognition_result = recognizer.recognize(mp_image)

    current_gesture_id = 0 # Default: nessun gesto riconosciuto

    if recognition_result.gestures:
        top_gesture = recognition_result.gestures[0][0]
        gesture_name = top_gesture.category_name
        
        # Mappatura dei gesti
        if gesture_name == "Closed_Fist":
            current_gesture_id = 1
        elif gesture_name == "Open_Palm":
            current_gesture_id = 2
        elif gesture_name == "Thumb_Up":
            current_gesture_id = 3
        elif gesture_name == "Victory":
            current_gesture_id = 4
            
        rospy.loginfo(f"Gesto Riconosciuto: {gesture_name} -> ID: {current_gesture_id}")

    # --- MODIFICA: Pubblica l'ID del gesto ad ogni frame ---
    msg_to_publish = Int32(data=current_gesture_id)
    gesture_publisher.publish(msg_to_publish)


def gesture_recognition_node():
    global gesture_publisher
    rospy.init_node('gesture_recognizer_node', anonymous=False)

    # Carica il percorso del modello dal parameter server
    default_model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gesture_recognizer.task')
    model_path = rospy.get_param('~model_path', default_model_path)

    if not init_gesture_recognizer(model_path):
        return

    # Publisher e Subscriber
    gesture_publisher = rospy.Publisher('/gesture', Int32, queue_size=1)
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1, buff_size=2**24)

    rospy.loginfo("Nodo 'gesture_recognizer_node' (versione semplificata) avviato.")
    rospy.loginfo(f"Sottoscritto a {image_topic} e pubblica su /gesture.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        gesture_recognition_node()
    except rospy.ROSInterruptException:
        pass