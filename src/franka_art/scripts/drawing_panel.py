import tkinter as tk
from tkinter import colorchooser, filedialog, font
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

class SimpleDrawingApp:
    def __init__(self, master):
        self.master = master
        master.title("App di Disegno Robotizzata Semplificata")

        # --- Inizializzazione ROS ---
        rospy.init_node('drawing_robot_controller', anonymous=True)
        self.rate_hz = 20 # Frequenza di pubblicazione in Hz
        self.pose_publisher = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(self.rate_hz) # Frequenza di pubblicazione di 20 Hz

        # --- Parametri Robot ---
        self.robot_fixed_x = 0.5 # Posizione fissa X (profondità) rispetto a franka_link0
        self.robot_y_min = -0.4 # Range Y del robot
        self.robot_y_max = 0.4
        self.robot_z_min = 0.1 # Range Z del robot
        self.robot_z_max = 0.7

        # Orientamento fisso del tool (perpendicolare al piano Y-Z)
        # Questo quaternion corrisponde a una rotazione di -90 gradi attorno all'asse Y globale,
        # facendo sì che l'asse Z del tool (direzione di lavoro) punti lungo l'asse Y globale.
        self.robot_orientation_quaternion = quaternion_from_euler(0, math.pi / 2, 0)

        # Posa Iniziale (Home Pose) del Robot
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = "panda_link0"
        self.home_pose.pose.position.x = self.robot_fixed_x
        self.home_pose.pose.position.y = 0.0 # Centro del range Y
        self.home_pose.pose.position.z = 0.4 # Centro del range Z
        self.home_pose.pose.orientation.x = self.robot_orientation_quaternion[0]
        self.home_pose.pose.orientation.y = self.robot_orientation_quaternion[1]
        self.home_pose.pose.orientation.z = self.robot_orientation_quaternion[2]
        self.home_pose.pose.orientation.w = self.robot_orientation_quaternion[3]

        # --- Stato dell'applicazione ---
        self.current_tool = "brush" # Strumento predefinito: pennello
        self.drawing = False # True quando il mouse è premuto
        self.start_x = None
        self.start_y = None
        self.drawing_active = False # True quando il conto alla rovescia è finito e si può disegnare
        self.countdown_timer = 0
        self.countdown_id = None # Per gestire l'ID del timer Tkinter

        # --- Configurazione della tela ---
        self.canvas_width = 800
        self.canvas_height = 600
        self.canvas = tk.Canvas(master, bg="white", width=self.canvas_width, height=self.canvas_height, relief=tk.SUNKEN, borderwidth=2)
        self.canvas.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.canvas.bind("<Button-1>", self.on_button_press)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release)

        # Etichetta per il conto alla rovescia
        self.countdown_label = self.canvas.create_text(self.canvas_width / 2, self.canvas_height / 2,
                                                       text="", font=("Arial", 72, "bold"), fill="gray", state=tk.HIDDEN)

        # --- Menu in alto (MenuBar) ---
        self.menubar = tk.Menu(master)
        master.config(menu=self.menubar)

        self.shape_menu = tk.Menu(self.menubar, tearoff=0)
        self.menubar.add_cascade(label="Strumenti", menu=self.shape_menu)
        # Solo pennello libero e gomma
        self.shape_menu.add_command(label="Pennello Libero", command=lambda: self.set_tool("brush"))
        self.shape_menu.add_command(label="Gomma per Cancellare", command=lambda: self.set_tool("eraser"))

        # --- Frame per il menu laterale dinamico ---
        self.sidebar_frame = tk.Frame(master, width=200, bg="#e0e0e0", relief=tk.RAISED, borderwidth=2)
        self.sidebar_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        # Pulsante di Avvio/Termina Disegno Robot
        self.start_stop_button = tk.Button(self.sidebar_frame, text="Avvia Disegno Robot",
                                           command=self.toggle_robot_drawing,
                                           font=("Arial", 12, "bold"), bg="#4CAF50", fg="white",
                                           activebackground="#45a049", activeforeground="white",
                                           relief=tk.RAISED, bd=3, padx=10, pady=5)
        self.start_stop_button.pack(pady=10, fill=tk.X)

        # Pulsante "Cancella Tutto"
        self.clear_all_button = tk.Button(self.sidebar_frame, text="Cancella Tutto",
                                          command=self.clear_canvas,
                                          font=("Arial", 12, "bold"), bg="#FF5722", fg="white",
                                          activebackground="#E64A19", activeforeground="white",
                                          relief=tk.RAISED, bd=3, padx=10, pady=5)
        self.clear_all_button.pack(pady=10, fill=tk.X)


        # Binding per la barra spaziatrice
        self.master.bind("<space>", lambda event: self.toggle_robot_drawing())

        # Opzioni di default per gli strumenti (solo pennello e gomma)
        self.options = {
            "brush": {"color": "black", "thickness": 5, "dash": ()}, # Spessore predefinito per il canvas
            "eraser": {"thickness": 10}
        }

        # Variabili di controllo per le opzioni della sidebar (solo tratto)
        # Il colore e lo spessore sono fissi per il pennello e la gomma
        self.var_dash = tk.StringVar(master, value="Solido")

        # Ora è sicuro chiamare create_sidebar_options
        self.create_sidebar_options()
        self.update_sidebar() # Inizializza la sidebar

        # Avvia il loop ROS/Tkinter
        self.ros_loop()

    def set_tool(self, tool_name):
        self.current_tool = tool_name
        self.update_sidebar()
        rospy.loginfo(f"Strumento selezionato: {self.current_tool}")

    def create_sidebar_options(self):
        self.widgets = {}

        # Opzioni Tratto (Line Style, solo per pennello)
        self.widgets["dash_label"] = tk.Label(self.sidebar_frame, text="Tratto:", bg="#e0e0e0")
        self.widgets["dash_optionmenu"] = tk.OptionMenu(self.sidebar_frame, self.var_dash, "Solido", "Tratteggiato", "Punteggiato")
        self.var_dash.trace_add("write", self.update_option_value)

        # Non ci sono più opzioni di colore o spessore variabili nella sidebar per pennello/gomma.
        # Lo spessore è fisso a 5 per il pennello e 10 per la gomma. Il colore del pennello è nero.

    def update_sidebar(self):
        # Nasconde tutti i widget nella sidebar
        for widget in self.widgets.values():
            widget.pack_forget()

        # Mostra solo i widget pertinenti allo strumento selezionato
        if self.current_tool == "brush":
            self.widgets["dash_label"].pack(pady=2)
            self.widgets["dash_optionmenu"].pack(pady=2)
            # Imposta i valori correnti dello strumento
            self.set_dash_value(self.options["brush"]["dash"])

        elif self.current_tool == "eraser":
            # La gomma non ha opzioni variabili nella sidebar in questa versione semplificata
            pass

    def update_option_value(self, *args):
        # Questo viene chiamato quando una variabile di controllo cambia.
        # Aggiorna il dizionario self.options con i nuovi valori.
        if self.current_tool == "brush":
            self.options["brush"]["dash"] = self.get_dash_tuple(self.var_dash.get())
        # La gomma non ha opzioni variabili nella sidebar in questa versione semplificata

    def get_dash_tuple(self, dash_str):
        if dash_str == "Solido":
            return ()
        elif dash_str == "Tratteggiato":
            return (5, 5)  # 5 pixel on, 5 pixel off
        elif dash_str == "Punteggiato":
            return (1, 3)  # 1 pixel on, 3 pixel off
        return ()

    def set_dash_value(self, dash_tuple):
        if dash_tuple == ():
            self.var_dash.set("Solido")
        elif dash_tuple == (5, 5):
            self.var_dash.set("Tratteggiato")
        elif dash_tuple == (1, 3):
            self.var_dash.set("Punteggiato")

    def toggle_robot_drawing(self):
        if not self.drawing_active:
            # Avvia il conto alla rovescia
            self.countdown_timer = 5
            self.canvas.itemconfig(self.countdown_label, text=str(self.countdown_timer), state=tk.NORMAL, fill="gray")
            self.start_stop_button.config(text="Termina Disegno Robot", bg="#f44336", activebackground="#d32f2f")
            rospy.loginfo("Avvio conto alla rovescia...")
            self.run_countdown()
        else:
            # Termina il disegno
            self.drawing_active = False
            if self.countdown_id:
                self.master.after_cancel(self.countdown_id) # Annulla il conto alla rovescia se ancora attivo
            self.canvas.itemconfig(self.countdown_label, state=tk.HIDDEN)
            self.start_stop_button.config(text="Avvia Disegno Robot", bg="#4CAF50", activebackground="#45a049")
            rospy.loginfo("Disegno robot terminato. Il robot tornerà alla posa iniziale.")
            self.publish_home_pose()

    def run_countdown(self):
        if self.countdown_timer > 0:
            self.canvas.itemconfig(self.countdown_label, text=str(self.countdown_timer))
            self.countdown_timer -= 1
            self.countdown_id = self.master.after(1000, self.run_countdown)
        else:
            self.canvas.itemconfig(self.countdown_label, text="VIA!", fill="green")
            self.master.after(500, lambda: self.canvas.itemconfig(self.countdown_label, state=tk.HIDDEN))
            self.drawing_active = True
            rospy.loginfo("Conto alla rovescia completato. Disegno attivo.")

    def publish_home_pose(self):
        """Pubblica la posa iniziale del robot."""
        self.home_pose.header.stamp = rospy.Time.now()
        self.pose_publisher.publish(self.home_pose)
        rospy.loginfo(f"Pubblicata posa iniziale: {self.home_pose.pose.position.y}, {self.home_pose.pose.position.z}")

    def on_button_press(self, event):
        self.drawing = True
        self.start_x = event.x
        self.start_y = event.y
        if self.current_tool == "brush" or self.current_tool == "eraser":
            # Colore e spessore fissi per il disegno sulla tela
            fill_color = self.options["brush"]["color"] if self.current_tool == "brush" else "white"
            outline_color = "" if self.current_tool == "brush" else "white"
            thickness = self.options[self.current_tool]["thickness"]
            self.canvas.create_oval(event.x - thickness/2,
                                    event.y - thickness/2,
                                    event.x + thickness/2,
                                    event.y + thickness/2,
                                    fill=fill_color, outline=outline_color,
                                    tags="current_drawing")

    def on_mouse_drag(self, event):
        if not self.drawing:
            return

        # Disegno sulla tela (sempre visibile)
        if self.current_tool == "brush":
            self.canvas.create_line(self.start_x, self.start_y, event.x, event.y,
                                     fill=self.options["brush"]["color"],
                                     width=self.options["brush"]["thickness"],
                                     capstyle=tk.ROUND, smooth=tk.TRUE, splinesteps=36,
                                     dash=self.options["brush"]["dash"])
            self.start_x = event.x
            self.start_y = event.y
        elif self.current_tool == "eraser":
            self.canvas.create_line(self.start_x, self.start_y, event.x, event.y,
                                     fill="white",
                                     width=self.options["eraser"]["thickness"],
                                     capstyle=tk.ROUND, smooth=tk.TRUE, splinesteps=36)
            self.start_x = event.x
            self.start_y = event.y

        # Pubblicazione della posa ROS (solo se il disegno robot è attivo e il mouse è premuto)
        if self.drawing_active and self.drawing:
            self.publish_robot_pose(event.x, event.y)

    def on_button_release(self, event):
        self.drawing = False
        self.start_x = None
        self.start_y = None

    def clear_canvas(self):
        """Cancella tutti gli elementi disegnati sulla tela."""
        self.canvas.delete("all")
        rospy.loginfo("Tela cancellata.")

    def publish_robot_pose(self, canvas_x, canvas_y):
        """
        Converte le coordinate della tela in coordinate robot e pubblica il messaggio PoseStamped.
        """
        # Mappatura delle coordinate della tela (pixel) alle coordinate del robot (metri)
        # L'asse Y del robot corrisponde all'asse X della tela
        robot_y = self.robot_y_min + (canvas_x / self.canvas_width) * (self.robot_y_max - self.robot_y_min)
        # L'asse Z del robot corrisponde all'asse Y della tela, ma invertito (0 in alto sulla tela è max Z per robot)
        robot_z = self.robot_z_max - (canvas_y / self.canvas_height) * (self.robot_z_max - self.robot_z_min)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "franka_link0" # Assicurati che questo sia il frame corretto del tuo robot

        pose_msg.pose.position.x = self.robot_fixed_x
        pose_msg.pose.position.y = robot_y
        pose_msg.pose.position.z = robot_z

        pose_msg.pose.orientation.x = self.robot_orientation_quaternion[0]
        pose_msg.pose.orientation.y = self.robot_orientation_quaternion[1]
        pose_msg.pose.orientation.z = self.robot_orientation_quaternion[2]
        pose_msg.pose.orientation.w = self.robot_orientation_quaternion[3]

        try:
            self.pose_publisher.publish(pose_msg)
            # rospy.loginfo(f"Pubblicata posa: X={pose_msg.pose.position.x:.3f}, Y={pose_msg.pose.position.y:.3f}, Z={pose_msg.pose.position.z:.3f}")
        except rospy.ROSException as e:
            rospy.logwarn(f"Impossibile pubblicare il messaggio ROS: {e}")

    def ros_loop(self):
        """
        Loop principale per la gestione degli eventi ROS e la pubblicazione.
        Integrato con Tkinter per non bloccare la GUI.
        """
        if not rospy.is_shutdown():
            # Questo loop è principalmente per garantire che il nodo ROS sia vivo.
            # La pubblicazione avviene in on_mouse_drag quando drawing_active è True.
            self.rate.sleep() # Pausa per mantenere la frequenza di pubblicazione desiderata
            self.master.after(int(1000 / self.rate_hz), self.ros_loop) # Ripianifica se stesso

if __name__ == "__main__":
    root = tk.Tk()
    app = SimpleDrawingApp(root)
    # Assicurati che il robot sia nella posa iniziale all'avvio dell'app
    app.publish_home_pose()
    root.mainloop()
