#include <iostream>
#include <string>
#include <mutex>
#include <cmath>
#include <queue>
#include <vector>
#include <tuple>
#include <fstream>
#include <algorithm>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

std::mutex battery_mutex;
std::mutex position_mutex;
std::mutex twist_mutex;

std::queue<std::tuple<double,double>> pointsToGo;

geometry_msgs::msg::Twist velocity;
geometry_msgs::msg::Point current_position;
geometry_msgs::msg::Quaternion quaternion_data;

float batteryState = 0.0;

bool isInKitchen = true;
bool first_run = true;

struct DeliveryInfo {
	std::string food_id;
	double x;
	double y;
};

struct Point {
    int x, y;
    double f, g, h; 
    Point* parent; 
    int intensity; 

    Point(int x, int y) : x(x), y(y), f(0), g(0), h(0), parent(nullptr), intensity(255) {}

    double calcularHeuristica(Point* goal) {
        // h = sqrt(pow((x - goal->x), 2) + pow((y - goal->y), 2));
        h = x - goal->x + y - goal->y;
        return h;
    }
};

std::queue<DeliveryInfo> deliveries_list;

std::tuple<double,double> toPos(int x, int y){
	float escala = 0.03;
	float OffsetX = -15.1;
	float OffsetY = -25.0;	
	double X, Y;
	
	X = OffsetX + escala*x;
	Y = -OffsetY - escala*y;
	std::tuple<double,double> posicao = std::make_tuple(X,Y);
	return posicao;
}

Point toIndex(double X, double Y){ // converte a posição real para indice na matriz
	float escala = 0.03;
	float OffsetX = -15.1;
	float OffsetY = -25.0;	
	int linha = -(Y + OffsetY)/escala;
	int coluna = (X - OffsetX)/escala;
	return Point(coluna, linha);
}

//This function asks the user to confirm that the food is on the robot

BT::NodeStatus IsFoodOnRobot() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food is on the Robot: ";
	
	while (std::stoi(number) < 1) {
		std::cin >> number;
		std::cout << std::endl;
	}

	return BT::NodeStatus::SUCCESS;
}

//This function checks if the robot's battery has more than 10% of power

BT::NodeStatus BatteryStatus() {
	battery_mutex.lock();
	if (batteryState <= 0.10) {
		battery_mutex.unlock();
		std::cout << "Battery is extremely low. The delivery has been cancelled!" << std::endl;
		return BT::NodeStatus::FAILURE;
	}
	battery_mutex.unlock();

	return BT::NodeStatus::SUCCESS;
}

//This function is responsible for confirming that the food has been picked up from the robot

BT::NodeStatus IsFoodTaken() {
	std::string number = "0";

	std::cout << "Press 1 or a greater number to confirm that the food has been picked up: ";
	
	while (std::stoi(number) < 1) {
		std::cin >> number;
		std::cout << std::endl;
	}

	return BT::NodeStatus::SUCCESS;
}

//This function verifies if the robot is on the kitchen

BT::NodeStatus IsRobotOnKitchen() {
	if ((current_position.x >= -0.5 && current_position.x <= 0.5) && (current_position.y >= -0.5 && current_position.y <= 0.5)){
		return BT::NodeStatus::SUCCESS;
	}

	return BT::NodeStatus::FAILURE;
}

class CalculatePath : public BT::SyncActionNode {
	private:
		bool dentroDosLimites(int x, int y, int linhas, int colunas) {
		    return (y >= 0 && y < linhas && x >= 0 && x < colunas);
		}

		bool dangerpos(std::vector<std::vector<int>>& mapa, int x, int y){
		    int margem = 10;
		    for (int i = -margem; i <= margem; i++){
			for (int j = -margem; j <= margem; j++){
			    if (!dentroDosLimites(x+i, y+j, mapa.size(), mapa[0].size())){
				return true;
			    }
			    if (mapa[y+i][x+j] == 1){
				return true;
			    }
			}
		    }
		    return false;
		    
		}

		std::vector<Point*> encontrarCaminhoAStar(std::vector<std::vector<int>>& mapa, Point* inicio, Point* objetivo) {
		    
		    if (!dentroDosLimites(inicio->x, inicio->y, mapa.size(), mapa[0].size())){
			std::cout << "Ponto inicial ffora do mapa!" << std::endl;
			return {};
		    }
		    
		    if (!dentroDosLimites(objetivo->x, objetivo->y, mapa.size(), mapa[0].size())){
			std::cout << "Ponto Final fora do mapa!" << std::endl;
			return {};
		    }

		    if (mapa[inicio->x][inicio->y] == 1){
			std::cout << "Ponto inicial na parede!" << std::endl;
			return {};
		    }
		    
		    if (mapa[objetivo->x][objetivo->y] == 1){
			std::cout << "Ponto Final na parede!" << std::endl;
			return {};
		    }


		    int direcoes[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};


		    auto comp = [](Point* a, Point* b) { return a->f > b->f; };
		    std::priority_queue<Point*, std::vector<Point*>, decltype(comp)> filaPrioridade(comp);

		    inicio->g = 0.0;
		    inicio->calcularHeuristica(objetivo);
		    inicio->f = inicio->g + inicio->h;
		    filaPrioridade.push(inicio);

		    while (!filaPrioridade.empty()) {
			Point* atual = filaPrioridade.top();

			filaPrioridade.pop();

			std::vector<Point*> caminho;
			if (atual->x == objetivo->x && atual->y == objetivo->y  ) {
			    for (Point* p = atual; p != nullptr; p = p->parent) {
				mapa[p->y][p->x] = 0;
				caminho.push_back(p);

			    }
			    reverse(caminho.begin(), caminho.end());
			    return caminho;
			}

			for (auto& dir : direcoes) {
			    int novoX = atual->x + dir[0];
			    int novoY = atual->y + dir[1];

			    

			    if (dentroDosLimites(novoX, novoY, mapa.size(), mapa[0].size()) && mapa[novoY][novoX] == 0) {
				if (!dangerpos(mapa, novoX, novoY)){
				    Point* vizinho = new Point(novoX, novoY);
				    vizinho->parent = atual;
				    vizinho->g = atual->g + 1; // Custo do movimento (assumindo custo 1 entre pontos adjacentes)
				    vizinho->calcularHeuristica(objetivo);
				    vizinho->f = vizinho->h + vizinho->g;
				    filaPrioridade.push(vizinho);
				    // Adiciona o vizinho à fila de prioridade
				}

				mapa[novoY][novoX] = -1; // Marcando como visitado
			    }
			}
		    }

		    return {};
		}

		std::vector<std::vector<int>> lerArquivoPGM(std::string nomeArquivo) {
		    std::ifstream arquivo(nomeArquivo, std::ios::binary);
		    std::vector<std::vector<int>> matriz;

		    if (!arquivo.is_open()) {
			std::cerr << "Erro ao abrir o arquivo." << std::endl;
			return matriz; // Retorna matriz vazia se houver erro na abertura do arquivo
		    }

		    // Variáveis para armazenar informações do cabeçalho PGM
		    std::string tipo;
		    int largura, altura, maxValor;

		    // Lê o tipo do arquivo PGM (deve ser 'P5')
		    std::getline(arquivo, tipo);
		    if (tipo != "P5") {
			std::cerr << "O arquivo não é um arquivo PGM no formato binário (P5)." << std::endl;
			arquivo.close();
			return matriz; // Retorna matriz vazia se não for PGM P5
		    }

		    // Pula comentários (linhas que começam com '#')
		    char c = arquivo.peek();
		    while (c == '#') {
			arquivo.ignore(256, '\n'); // Ignora a linha de comentário
			c = arquivo.peek();
		    }

		    // Lê largura, altura e valor máximo de intensidade
		    arquivo >> largura >> altura >> maxValor;
		    arquivo.get(); // Para consumir o caractere de quebra de linha após o valor máximo

		    // Lê os pixels do arquivo PGM e cria a matriz binária
		    matriz.resize(altura, std::vector<int>(largura));

		    // Lê os pixels byte a byte
		    for (int i = 0; i < altura; ++i) {
			for (int j = 0; j < largura; ++j) {
			    unsigned char valorPixel;
			    arquivo.read(reinterpret_cast<char*>(&valorPixel), 1);

			    // Determina se o pixel é branco ou não (usando um limiar de 255/2 = 127.5)
			    matriz[i][j] = (valorPixel < 250) ? 1 : 0;
			}
		    }

		    arquivo.close();
		    return matriz;
		}

	public:
		CalculatePath(const std::string& name)
			: BT::SyncActionNode(name, {}) {}

		BT::NodeStatus tick() override {
			std::cout << "Creating a path to the desired location..." << std::endl;

			std::string nomeArquivo = "src/delivery_robot/src/bt_folder/my_map.pgm"; 

			std::vector<std::vector<int>> mapa = lerArquivoPGM(nomeArquivo);

    			//int linhas = mapa.size();
    			//int colunas = mapa[0].size();
			
			Point inicio(0, 0);
			Point objetivo(0, 0);
			
			if (isInKitchen && first_run) {
				inicio = toIndex(0.0, 0.0);

				DeliveryInfo data = deliveries_list.front();
				objetivo = toIndex(data.x, data.y);
			}else if (isInKitchen && !first_run) {
    				inicio = toIndex(current_position.x, current_position.y);

				DeliveryInfo data = deliveries_list.front();
				objetivo = toIndex(data.x, data.y);
			}else{
				inicio = toIndex(current_position.x, current_position.y);
				objetivo = toIndex(0.0, 0.0);
			}
    
			std::vector<Point*> caminho = encontrarCaminhoAStar(mapa, &inicio, &objetivo);
			std::cout << "Current pos: " << current_position.x << " " << current_position.y << std::endl;

			for (auto it = caminho.begin(); it != caminho.end(); it++){
				Point* point = *it;
				pointsToGo.push(toPos(point->x, point->y));
				std::tuple<double, double> ponto = pointsToGo.back();
				std::cout << std::get<0>(ponto) << " " << std::get<1>(ponto) << std::endl;
			}
    
    			if (!caminho.empty()) {
				std::cout << "Path to patient room was defined!" << std::endl;
    			} else {
				std::cout << "Can't define a path to the desired location." << std::endl;
				return BT::NodeStatus::FAILURE;
    			}
			
			return BT::NodeStatus::SUCCESS;
		}
};

//This node is resposible for getting user input about the delivery

class RegisterDeliveryInfo : public BT::SyncActionNode {
	public:
		RegisterDeliveryInfo(const std::string& name, const BT::NodeConfiguration& config)
		       : BT::SyncActionNode(name, config) {}
		
		static BT::PortsList providedPorts() {
			return {BT::OutputPort<int>("given_deliveries")};
		}

		BT::NodeStatus tick() override {
			std::string food_id = "";
			std::string x = "0.0";
			std::string y = "0.0";
			std::string deliveries_num = "0";
			
			std::cout << "Type the number of food items that you want to deliver: " << std::endl;

			while (std::stoi(deliveries_num) <= 0) {
				std::cin >> deliveries_num;
			}

			for (int i = 0; i<std::stoi(deliveries_num); i++) {
				std::cout << "Type the food identification of your desire: ";
				std::cin >> food_id;

				std::cout << std::endl << "Now type the x coordinate of the room: ";
				std::cin >> x;

				std::cout << std::endl << "Now type the y coordinate of the room: ";
				std::cin >> y;
				std::cout << std::endl << std::endl;

				DeliveryInfo info = {food_id, std::stod(x), std::stod(y)};

				deliveries_list.push(info);
			}

			setOutput("given_deliveries", std::stoi(deliveries_num));

			return BT::NodeStatus::SUCCESS;
		}
};

//This node is responsible for the movement of the robot using a PID controller

class GoToPatientRoom : public BT::StatefulActionNode {
	private:
		double targetX = 0.0;
		double targetY = 0.0;
		double current_linear_error_ = 0.0;
		double current_angular_error_ = 0.0;
		double linear_error_sum_ = 0.0;
		double angular_error_sum_ = 0.0;
		const double KP_linear_ = 0.05;
		const double KI_linear_ = 0.01;
		const double KP_angular_ = 0.005;
		const double KI_angular_ = 0.001;

		void calculate_error() {
			position_mutex.lock();
			double linear_error = sqrt((pow((this -> targetX - current_position.x), 2.0)) + pow((this -> targetY - current_position.y), 2.0));
			
			double target_angle = atan2(this -> targetY - current_position.y, this -> targetX - current_position.x);
			
			double current_angle = atan2(2.0*(quaternion_data.y*quaternion_data.x + 
					quaternion_data.w*quaternion_data.z), 1.0 - 2.0*(pow(quaternion_data.z, 2.0) +
					pow(quaternion_data.y, 2.0)));
			position_mutex.unlock();

			double angular_error = target_angle - current_angle;
			
			this -> current_linear_error_ = linear_error;
			this -> current_angular_error_ = angular_error;
			this -> linear_error_sum_ += linear_error;
			this -> angular_error_sum_ += angular_error;
		}

		double calculate_linear_velocity() {
			return (KP_linear_*current_linear_error_) + (KI_linear_*linear_error_sum_);
		}

		double calculate_angular_velocity() {
			return (KP_angular_*current_angular_error_) + (KI_angular_*angular_error_sum_);
		}
	
	public:
		GoToPatientRoom(const std::string& name)
			: BT::StatefulActionNode(name, {}) {}

		BT::NodeStatus onStart() override {
			std::tuple<double,double> data = pointsToGo.front();
			this -> targetX = std::get<0>(data);
			this -> targetY = std::get<1>(data);

			std::cout << "Initializing delivery process..." << std::endl;

			return BT::NodeStatus::RUNNING;
		}

		BT::NodeStatus onRunning() override {
			this -> calculate_error();
			
			if (abs(this -> current_angular_error_) > 0.3) {
				twist_mutex.lock();
				velocity.angular.z = this -> calculate_angular_velocity();
				velocity.linear.x = 0.0;
				twist_mutex.unlock();
			}else if (abs(this -> current_linear_error_) > 0.1) {
				twist_mutex.lock();
				velocity.angular.z = 0.0;
				velocity.linear.x = this -> calculate_linear_velocity();
				twist_mutex.unlock();
			}else{
				twist_mutex.lock();
				velocity.linear.x = 0.0;
				velocity.linear.z = 0.0;
				twist_mutex.unlock();
			
				this -> current_linear_error_ = 0.0;
				this -> current_angular_error_ = 0.0;
				this -> linear_error_sum_ = 0.0;
				this -> angular_error_sum_ = 0.0;
				
				pointsToGo.pop();
				if (!pointsToGo.empty()) {
					std::tuple<double,double> next_point = pointsToGo.front();
					this -> targetX = std::get<0>(next_point);
					this -> targetY = std::get<1>(next_point);
					std::cout << "Arrived at point" << std::endl;
					return BT::NodeStatus::RUNNING;
				}

				isInKitchen = false;

				return BT::NodeStatus::SUCCESS;
			}

			return BT::NodeStatus::RUNNING;
		}

		void onHalted() override {
			std::cout << "The process has been interrupted!" << std::endl; 
		}
};

//This node is responsible for displaying information about the delivery

class DisplayFoodInfo : public BT::SyncActionNode {
	public:
		DisplayFoodInfo(const std::string& name) 
			: BT::SyncActionNode(name, {}) {}

		BT::NodeStatus tick() override {
			DeliveryInfo data = deliveries_list.front();
			std::cout << "The food has arrived! " << std::endl << "Take the food with this id: " << data.food_id << std::endl;
			
			return BT::NodeStatus::SUCCESS;
		}
};

//This node updates data to set the robot for the next delivery, if there is any

class UpdateDeliveryInfo : public BT::SyncActionNode {
	public:
		UpdateDeliveryInfo(const std::string& name)
			: BT::SyncActionNode(name, {}) {}

		BT::NodeStatus tick() override {
			if (deliveries_list.empty()) {
				return BT::NodeStatus::SUCCESS;
			}

			deliveries_list.pop();
			
			return BT::NodeStatus::SUCCESS;
		}
};

//This node is responsible for making the robot go back to the kitchen

class GoBackToKitchen : public BT::StatefulActionNode {
	private:
		double targetX = 0.0;
		double targetY = 0.0;
		double current_linear_error_ = 0.0;
		double current_angular_error_ = 0.0;
		double linear_error_sum_ = 0.0;
		double angular_error_sum_ = 0.0;
		const double KP_linear_ = 0.05;
		const double KI_linear_ = 0.01;
		const double KP_angular_ = 0.005;
		const double KI_angular_ = 0.001;

		void calculate_error() {
			position_mutex.lock();
			double linear_error = sqrt((pow((this -> targetX - current_position.x), 2.0)) + pow((this -> targetY - current_position.y),2.0));
			
			double target_angle = atan2(this -> targetY - current_position.y, this -> targetX - current_position.x);
			
			double current_angle = atan2(2.0*(quaternion_data.y*quaternion_data.x + 
					quaternion_data.w*quaternion_data.z), 1.0 - 2.0*(pow(quaternion_data.z, 2.0) +
					pow(quaternion_data.y, 2.0)));
			position_mutex.unlock();

			double angular_error = target_angle - current_angle;
			
			this -> current_linear_error_ = linear_error;
			this -> current_angular_error_ = angular_error;
			this -> linear_error_sum_ += linear_error;
			this -> angular_error_sum_ += angular_error;
		}

		double calculate_linear_velocity() {
			return (KP_linear_*current_linear_error_) + (KI_linear_*linear_error_sum_);
		}

		double calculate_angular_velocity() {
			return (KP_angular_*current_angular_error_) + (KI_angular_*angular_error_sum_);
		}
	
	public:
		GoBackToKitchen(const std::string& name)
			: BT::StatefulActionNode(name, {}) {}

		BT::NodeStatus onStart() override {
			std::cout << "Going back to kitchen..." << std::endl;
			
			std::tuple<double,double> data = pointsToGo.front();
			this -> targetX = std::get<0>(data);
			this -> targetY = std::get<1>(data);

			return BT::NodeStatus::RUNNING;
		}

		BT::NodeStatus onRunning() override {
			this -> calculate_error();
			
			if (abs(this -> current_angular_error_) > 0.3) {
				twist_mutex.lock();
				velocity.angular.z = this -> calculate_angular_velocity();
				velocity.linear.x = 0.0;
				twist_mutex.unlock();
			}else if (abs(this -> current_linear_error_) > 0.1) {
				twist_mutex.lock();
				velocity.angular.z = 0.0;
				velocity.linear.x = this -> calculate_linear_velocity();
				twist_mutex.unlock();
			}else{
				twist_mutex.lock();
				velocity.linear.x = 0.0;
				velocity.linear.z = 0.0;
				twist_mutex.unlock();

				this -> current_linear_error_ = 0.0;
				this -> current_angular_error_ = 0.0;
				this -> linear_error_sum_ = 0.0;
				this -> angular_error_sum_ = 0.0;
				
				pointsToGo.pop();
				if (!pointsToGo.empty()) {
					std::tuple<double,double> next_point = pointsToGo.front();
					this -> targetX = std::get<0>(next_point);
					this -> targetY = std::get<1>(next_point);
					std::cout << "Arrived at point" << std::endl;
					return BT::NodeStatus::RUNNING;
				}

				isInKitchen = true;

				return BT::NodeStatus::SUCCESS;
			}

			return BT::NodeStatus::RUNNING;
		}

		void onHalted() override {
			std::cout << "The process has been interrupted!" << std::endl;
		}

};
