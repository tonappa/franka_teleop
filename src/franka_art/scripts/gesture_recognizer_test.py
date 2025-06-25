#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridgeError, CvBridge
import cv2
import mediapipe as mp
import os
import time

# Inizializzazione di CvBridge per la conversione delle immagini
bridge = CvBridge()

# Inizializzazione del MediaPipe Gesture Recognizer
recognizer = None

# Variabili per la gestione dello stato e del debounce
last_published_gesture_id = -1 # Inizializzato a -1 per garantire la prima pubblicazione al primo gesto stabile
prev_recognized_gesture_id = 0  # Il gesto riconosciuto nel frame precedente (prima del debounce)
consecutive_frames_count = 0    # Contatore di frame consecutivi per il riconoscimento stabile

# Regola questo valore per più o meno reattività vs. stabilità
# 1 = reattività massima (ma meno stabile), 3 = default (buon compromesso)
min_consecutive_frames = 2      # Ho abbassato a 2 per maggiore reattività

# Variabile per il heartbeat (solo per la vitalità del nodo, non per log ogni volta)
last_heartbeat_publish_time = time.time()
heartbeat_rate_hz = 1 # Pubblica il gesto corrente ogni 1 secondo per "heartbeat"

def init_gesture_recognizer(model_path):
    """Inizializza il riconoscitore di gesti di MediaPipe."""
    global recognizer
    try:
        if not os.path.exists(model_path):
            rospy.logerr(f"Errore: Il file del modello non esiste al percorso: {model_path}")
            rospy.signal_shutdown("Modello di riconoscimento gesti non trovato.")
            return False

        base_options = mp.tasks.BaseOptions(model_asset_path=model_path)
        
        # --- OPZIONI AGGIUNTE PER MIGLIORARE LA REATTIVITÀ ---
        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=base_options,
            num_hands=1,  # Rileva al massimo una mano, per concentrarsi su una sola
            # Aumenta la confidenza minima di rilevamento/tracciamento se i falsi positivi sono un problema
            # o abbassala per rilevare mani più difficili (potrebbe aumentare falsi positivi)
            min_hand_detection_confidence=0.5, # Default è 0.5, puoi provare ad abbassare leggermente se necessario
            min_tracking_confidence=0.5, # Default è 0.5, puoi provare ad abbassare leggermente se necessario
            # min_hand_presence_confidence=0.5 # Aggiunto in alcune versioni, confidenza che una mano sia presente
        )
        # ----------------------------------------------------

        recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        rospy.loginfo(f"MediaPipe Gesture Recognizer caricato con successo da: {model_path}")
        return True
    except Exception as e:
        rospy.logerr(f"Errore nel caricamento del modello MediaPipe: {e}")
        rospy.signal_shutdown("Impossibile caricare il modello di riconoscimento gesti.")
        return False

def image_callback(msg, publisher):
    """Callback per i messaggi di immagine."""
    global last_published_gesture_id, prev_recognized_gesture_id, consecutive_frames_count, last_heartbeat_publish_time

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv_image)
    recognition_result = recognizer.recognize(mp_image)

    current_raw_gesture_id = 0 # Gesto riconosciuto nel frame attuale, senza debounce

    if recognition_result.gestures:
        top_gesture = recognition_result.gestures[0][0]
        gesture_name = top_gesture.category_name

        # Mappatura dei gesti ai valori Int32
        if gesture_name == "Closed_Fist":
            current_raw_gesture_id = 1 # Pugno Chiuso -> Grasp
        elif gesture_name == "Open_Palm":
            current_raw_gesture_id = 2 # Palmo Aperto -> Open Gripper
        elif gesture_name == "Thumb_Up":
            current_raw_gesture_id = 3 # Pollice Su -> Comando di impostazione 1
        elif gesture_name == "Victory":
            current_raw_gesture_id = 4 # Gesto Vittoria -> Comando di impostazione 2
        # Se un altro gesto è riconosciuto o nessun gesto, current_raw_gesture_id rimane 0

    # --- Logica di Debounce per stabilizzare il riconoscimento ---
    if current_raw_gesture_id == prev_recognized_gesture_id:
        consecutive_frames_count += 1
    else:
        consecutive_frames_count = 1
        prev_recognized_gesture_id = current_raw_gesture_id

    # --- Logica di Pubblicazione al Cambio di Stato Stabile ---
    # Se il gesto corrente è stabile E diverso dall'ultimo pubblicato
    if consecutive_frames_count >= min_consecutive_frames and \
       current_raw_gesture_id != last_published_gesture_id:
        
        msg_to_publish = Int32(data=current_raw_gesture_id)
        publisher.publish(msg_to_publish)
        last_published_gesture_id = current_raw_gesture_id # Aggiorna l'ultimo ID pubblicato
        last_heartbeat_publish_time = time.time() # Resetta il timer heartbeat dopo una pubblicazione di cambio stato
        
        # Output sul terminale solo al cambio di stato stabile
        rospy.loginfo(f"*** PUBBLICA: Nuovo Gesto Riconosciuto Stabile: ID {current_raw_gesture_id} ***")

    # --- Logica di Heartbeat (Invio Periodico dell'Ultimo Stato Pubblicato) ---
    current_time = time.time()
    if (current_time - last_heartbeat_publish_time) >= (1.0 / heartbeat_rate_hz):
        # Invia l'ultimo stato noto, per mantenere il robot informato della vitalità del nodo e dello stato
        msg_to_publish = Int32(data=last_published_gesture_id)
        publisher.publish(msg_to_publish)
        last_heartbeat_publish_time = current_time
        # Nessun log per l'heartbeat, per mantenere pulito il terminale

def gesture_recognition_node():
    rospy.init_node('gesture_recognizer_node', anonymous=False)

    model_path = rospy.get_param('~model_path', os.path.join(os.path.dirname(__file__), 'gesture_recognizer.task'))

    if not init_gesture_recognizer(model_path):
        return

    gesture_publisher = rospy.Publisher('/gesture', Int32, queue_size=10)

    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback, (gesture_publisher))

    rospy.loginfo(f"Nodo 'gesture_recognizer_node' avviato, sottoscritto a {image_topic} e pubblica su /gesture.")
    rospy.loginfo(f"I comandi vengono pubblicati solo al **cambio di stato stabile** (dopo {min_consecutive_frames} frame).")
    rospy.loginfo(f"Lo stato corrente (ID 0 per nessun gesto) viene ribadito ogni {1.0 / heartbeat_rate_hz:.1f} secondi (heartbeat senza log).")
    rospy.loginfo("Iniziare i gesti per controllare il robot.")

    rospy.spin()

if __name__ == '__main__':
    try:
        gesture_recognition_node()
    except rospy.ROSInterruptException:
        pass