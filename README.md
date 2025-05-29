# RPproject_robotic_arm
Il progetto riguarda lo sviluppo di un Nodo ROS2 in C++ per la simulazione di un braccio robotico a 5 joints (5R) con calcolo della cinematica inversa tramite uso della pseudo-inversa della Jacobiana numerica ed animazione del movimento in RViz.

# Struttura
robotic_arm/                        # package ROS2
- CMakeLists.txt                    # build instructions
- package.xml                       # descrizione del package e dipendenze
- src/robot_invkinematics.cpp       # nodo IK (IKNode) in C++
- config/dh_parameters.yaml         # configurazione parametri Denavit–Hartenberg
- urdf/robotic_arm.urdf             # file URDF per la descrizione geometrica del robot in Rviz
- launch/start_simulation.launch.py # avvia joint_state_publisher, robot_state_publisher, ik_node e la simulazione in RViz
- rviz/configuration.rviz           # layout della visualizzazione su Rviz

# Esecuzione
- Chiamare il launch file in un terminale (disabilitando la finestra GUI con sliders dei joints) --> ros2 launch robotic_arm start_simulation.launch.py jsp_gui:=false
- Attendere qualche secondo perchè il nodo e Rviz si mettano "in ascolto"
- Inviare la target position da un altro terminale --> ros2 topic pub --once /target_position std_msgs/msg/Float64MultiArray "{data: [0.3, 0.8, 0.8]}"

Il risultato del calcolo della cinematica inversa dovrebbe apparire sul primo terminale, mentre il robot dovrebbe muoversi su Rviz fino a raggiungere la posizione desiderata con l'end-effector (tramite interpolazione della posa corrente con quella calcolata tramite Inverse Kinematics).
