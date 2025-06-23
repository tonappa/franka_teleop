#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

def gesture_callback(result: vision.GestureRecognizerResult,
                     output_image: mp.Image,
                     timestamp_ms: int,
                     publisher: rospy.Publisher):
    """
    Callback invocata da MediaPipe ogni volta che
    riconosce un gesto in un frame.
    """
    # result.gestures è una lista di liste di Category
    if result.gestures:
        # prendi il primo gesto della prima mano
        top_gesture = result.gestures[0]
        if top_gesture:
            name = top_gesture[0].category_name
            publisher.publish(name)
            rospy.loginfo(f"[Gesture] {name}")

def main():
    rospy.init_node('gesture_recognizer')
    pub = rospy.Publisher('gesture_recognition', String, queue_size=10)

    # --- Setup MediaPipe Gesture Recognizer ---
    BaseOptions = mp.tasks.BaseOptions
    GestureRecognizer = mp.tasks.vision.GestureRecognizer
    GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
    RunningMode = mp.tasks.vision.RunningMode

    # Inserisci qui il percorso assoluto del modello .task di MediaPipe
    model_path = '/home/tonappa/Desktop/franka_teleop/src/franka_art/models/gesture_recognizer.task'

    options = GestureRecognizerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=RunningMode.LIVE_STREAM,
        result_callback=lambda res, img, ts: gesture_callback(res, img, ts, pub)
    )

    with GestureRecognizer.create_from_options(options) as recognizer:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            rospy.logerr("Impossibile aprire la webcam")
            return

        rospy.loginfo("Avvio riconoscimento gesti. Premi CTRL+C per uscire.")
        rate = rospy.Rate(30)  # tenterà ~30 fps
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("Frame non ricevuto dalla webcam")
                break

            # Converti BGR (OpenCV) → RGB (MediaPipe)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

            # timestamp in ms per media timestamp (cv2.CAP_PROP_POS_MSEC)
            timestamp = int(cap.get(cv2.CAP_PROP_POS_MSEC))
            recognizer.recognize_async(mp_image, timestamp)

            rate.sleep()

        cap.release()
    rospy.loginfo("Nodo gesture_recognizer terminato.")

if __name__ == '__main__':
    main()
