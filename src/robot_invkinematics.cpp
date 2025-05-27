#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

using namespace std::chrono_literals; // per usare suffissi temporali (es. 100ms)
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Struttura per i parametri Denavit-Hartenberg 
struct DHParam { double theta, d, a, alpha; };

class IKNode : public rclcpp::Node {
public:
  // inizializzo il nodo 
  IKNode() : Node("ik_node") {
  
    // Percorso assoluto del file YAML dei parametri DH 
    std::string dh_path = ament_index_cpp::get_package_share_directory("robotic_arm")
                             + "/config/dh_parameters.yaml";
    // Dichiarazione il parametro "dh_params_file" e lo leggo 
    this->declare_parameter<std::string>("dh_params_file", dh_path);
    std::string dh_file;
    this->get_parameter("dh_params_file", dh_file);
    
    RCLCPP_INFO(this->get_logger(), "Carico parametri DH da: %s", dh_file.c_str());
    
     // Caricamento dei parametri DH da file
    loadDH(dh_file);    
    
    // Inizializzo la posa di partenza (tutti zeri)
	last_solution_ = Eigen::VectorXd::Zero(dh_.size());

	// Publisher per i JointState su '/joint_states'
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // pubblico la posa iniziale
    //publishJointState(last_solution_);
    
    // Timer per interpolazione --> publishNextStep() ogni 100ms
    timer_ = this->create_wall_timer(
	  100ms, [this]() { publishNextStep(); }
	);
	
	// Sub al topic '/target_position' per ricevere posizioni target
	target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/target_position", 10, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg){ onTarget(msg->data); });  
    
    RCLCPP_INFO(get_logger(), "IK nodo inizializzato.");
  }

private:
  std::vector<DHParam> dh_;   // Vettore di 5 parametri DH
  // inizializzazione degli altri vettori
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  VectorXd last_solution_; // ultima posa calcolata
  std::vector<Eigen::VectorXd> trajectory_;   // sequenza di step --> configurazioni interpolate
  size_t traj_index_ = 0;  // indice corrente
  rclcpp::TimerBase::SharedPtr timer_;  // timer per animazione

  // ------------------------------------------//
  // FUNZIONE per caricamento dei parametri DH da YAML --> Legge le entry da "joint_1" a "joint_5" e popola dh_
  void loadDH(const std::string &path) {
    auto doc = YAML::LoadFile(path)["dh_parameters"];
    for (int i = 1; i <= 5; ++i) {
      auto v = doc["joint_" + std::to_string(i)].as<std::vector<double>>();
      dh_.push_back({v[0], v[1], v[2], v[3]}); // {v[0], v[1], v[2], v[3]} = {theta, d, a, alpha}
    }
    RCLCPP_INFO(get_logger(), "Caricati %zu parametri DH.", dh_.size());
  }
  // ------------------------------------------//
	
  // ------------------------------------------//
  // FUNZIONE per cinematica diretta --> posizione end-effector da q (configurazione di joints)
  Vector3d forwardKinematics(const VectorXd &q) {
    Matrix4d T = Matrix4d::Identity();  // Inizializzo la matrice di trasformazione T come identità 4×4 
    for (int i = 0; i < q.size(); ++i) { 
      double θ = q[i] + dh_[i].theta; // Calcolo l’angolo theta sommando l’offset dh_[i].theta (definito nel file DH) a q[i] angolo nella configurazione attuale
      double d = dh_[i].d; 		// estraggo d (displacement lungo z)
      double a = dh_[i].a; 		// estraggo a (displacement lungo x)
      double α = dh_[i].alpha;  // estraggo alpha (rotazione attorno ad x)
      // Costruisco la matrice di trasformazione omogenea A_i dal link i-1 a i
      Matrix4d A;  
      A << cos(θ), -sin(θ), 0, a,
           sin(θ)*cos(α), cos(θ)*cos(α), -sin(α), -sin(α)*d,
           sin(θ)*sin(α), cos(θ)*sin(α),  cos(α),  cos(α)*d,
           0,            0,             0,       1;
      T = T * A;  // concatenazione delle trasformazioni
    }
    // estraggo la colonna di traslazione [x,y,z] che rappresenta 
    // la posizione dell'end effector wrt il base frame
    return Vector3d(T(0,3), T(1,3), T(2,3));
  }
  // ------------------------------------------//
  
  // ------------------------------------------//
  // FUNZIONE per calcolo della jacobiana numerica 
  MatrixXd numericJacobian(const VectorXd &q, double δ=1e-6) {
    Vector3d p0 = forwardKinematics(q);  // calcolo pos dell'end effector
    int n = q.size();
    MatrixXd J(3,n);  // inizializzo J, matrice 3xN dove ciascuna colonna j conterrà ∂p/∂q
    for (int j = 0; j < n; ++j) {
      VectorXd qp = q;  // nuovo vettore
      qp[j] += δ;       // con piccolo incremento su q_j
      Vector3d p1 = forwardKinematics(qp);  // calcolo pos dell'end effector con questo incremento
      J.col(j) = (p1 - p0) / δ;   // derivata approssimata calcolata tramite differenze finite
    }
    return J;
  }
  // ------------------------------------------//

  // ------------------------------------------//
  // FUNZIONE per la cinematica inversa IK (iterativa) usando la pseudo-inversa della jacobiana
  // dato un target, trova la posizione dei giunti (configurazione)
  Eigen::VectorXd solveIK(const Vector3d &target) {   //std::vector<Eigen::VectorXd>
    int n = dh_.size();
    //std::vector<VectorXd> traj; per prova con tutta la traiettoria
    //traj.reserve(100);
    VectorXd q = VectorXd::Zero(n);  // inizializzo configurazione a zero
    double lr = 0.5;  // learning rate
    double eps = 1e-4; // bound per la convergenza dell'errore
    for (int iter = 0; iter < 100; ++iter) {
      Vector3d err = target - forwardKinematics(q);  // errore tra posizione target e attuale
      if (err.norm() < eps) break;  // controllo di convergenza
      MatrixXd J    = numericJacobian(q);  // calcolo J
      MatrixXd Jpinv= J.completeOrthogonalDecomposition().pseudoInverse();  // pseudo inversa
      q += lr * (Jpinv * err);  // passo di aggiornamento
      //traj.push_back(q);  // salva la configurazione intermedia
    }
    // ritorna solo la posa finale qf che minimizza l'errore
    return q; //traj;
  }
  // ------------------------------------------//
  
  // ------------------------------------------//
  // FUNZIONE per pubblicare JointState 
  void publishJointState(const Eigen::VectorXd &q) {
	sensor_msgs::msg::JointState js;
	js.header.stamp = this->get_clock()->now();
	js.name = {"joint_1","joint_2","joint_3","joint_4","joint_5"};
	js.position.assign(q.data(), q.data()+q.size());
	joint_pub_->publish(js);
  }
  
  // FUNZIONE chiamata nel timer per pubblicare passo-passo la traiettoria interpolata
  void publishNextStep() {
	if (traj_index_ < trajectory_.size()) {
		// pubblica il passo corrente di interpolazione
		publishJointState(trajectory_[traj_index_++]);
	}
	else {
		// una volta finiti i passi, continua a pubblicare la posa finale (così non salta nuovamente a quella iniziale)
		publishJointState(last_solution_);
	}
  }
  // ------------------------------------------//


  // QUANDO RICEVE IL MESSAGGIO DI TARGET
  // funzione callback chiamata quando viene ricevuto un messaggio 
  // sul topic '/target_position' a cui il nodo è sottoscritto
  void onTarget(const std::vector<double> &data) {
    if (data.size() < 3) return;  //controllo
    Vector3d tgt(data[0], data[1], data[2]);
    
    VectorXd q0 = last_solution_; // posa di partenza (iniziale o quella del target recedente)
    VectorXd qf = solveIK(tgt); // posa finale calcolata tramite "solveIK"
    
    // piccola sezione per normalizzare la differenza angolare al range [-pi,pi]
    // serve per evitare che, essendo jint di tipo continuous, avvengano salti 
    // per raggiungere posizioni del giunto in realtà vicine 
    VectorXd delta = qf - q0;  // displacement fra una configurazione e l'altra
	for (int i = 0; i < delta.size(); ++i) {  // per ogni giunto controlla e normalizza
	  while (delta[i] > M_PI)  delta[i] -= 2*M_PI;
	  while (delta[i] < -M_PI) delta[i] += 2*M_PI;
	}

    // qui costruisco le configurazioni interpolate tra la partenza e l'arrivo
    // per simulare il movimeto del robot
    const size_t N = 20;  // numero di frame
	trajectory_.clear();
	trajectory_.reserve(N+1);  // 20 configurazioni + iniziale
	for (size_t k = 0; k <= N; ++k) {
		double α = double(k)/N;  // percentuale di avvicinamento fra 0 e 1 alla posa finale
		trajectory_.push_back(q0 + α*delta); //aggiunge in coda l'interpolazione lineare --> (q0 * (1 - α) + qf * α);
	}
    
    // reset indice e aggiorna ultima soluzione
    traj_index_ = 0;
	last_solution_ = qf;  //trajectory_.back();
    
	RCLCPP_INFO(get_logger(),
				"Interpolazione: %zu frame da [% .3f,…] a [% .3f,…]",
				trajectory_.size(), q0[0], qf[0]);
   }
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKNode>());
  rclcpp::shutdown();
  return 0;
}

