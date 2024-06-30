#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <fstream>
#include <algorithm>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"


using namespace std;

// Obs é preciso configurar a locallização do mapa. 
// Por padrão, o programa vai tentar ler o arquivo my_map.pgm,
// mapa completo do warehouse, que deve estar nesse diretório.
// Caso utilize outro mapa, modifique os parametros nos dados do mapa (nome, escala e Offset's)
// Falta tambem coletar as posições do robo e do local de destino, por enquanto está apenas para colocar manualmente



class CalculatePath : public BT::StatefulActionNode {
	private:
		struct Point {
			int x, y;
			double f, g, h; 
			Point* parent; 

			Point(int x, int y) : x(x), y(y), f(0), g(0), h(0), parent(nullptr) {}

			// Função de cálculo da distância heurística (Distancia Manhattan ou Euclidiana)
			double calcularHeuristica(Point* goal) {
				// h = sqrt(pow((x - goal->x), 2) + pow((y - goal->y), 2));	// Euclidiana
				h = x - goal->x + y - goal->y;					// Manhattan
				return h;
			}
		};


		
		// Posição real
		double targetX = 0.0; 
		double targetY = 0.0;

		double robotX = 0.0;
		double robotY = 0.0;
		
		// Dados do mapa (Obs: falta implementar buscar esses dados no my_map.yaml)
		string nomeArquivo = "my_map.pgm"; 
		float escala = 0.03;
		float OffsetX = -15.1;
		float OffsetY = -25.0;



		Point* inicio;
		Point* objetivo;

		vector<vector<int>> mapa;
		vector<Point*> caminho;
		vector<geometry_msgs::msg::Point*> finalpath;

		
		
		bool dentroDosLimites(int x, int y, int linhas, int colunas) {
 		   return (y >= 0 && y < linhas && x >= 0 && x < colunas);
		}

		bool dangerpos(vector<vector<int>>& mapa, int x, int y, int margem = 10){
			for (int i = -margem; i <= margem; i++){
				for (int j = -margem; j <= margem; j++){
					if (!dentroDosLimites(x+i, y+j, mapa.size(), mapa[0].size()) || mapa[y+i][x+j] == 1){
						return true;
					}
				}
			}
			return false;
		}

		vector<vector<int>> GetMap(string nomeArquivo) {
			ifstream arquivo(nomeArquivo, ios::binary);
			vector<vector<int>> matriz;

			if (!arquivo.is_open()) {
				cerr << "Erro ao abrir o arquivo." << endl;
				return matriz;
			}

			int largura, altura, maxValor;

			getline(arquivo);
			arquivo >> largura >> altura >> maxValor;
			arquivo.get(); 

			matriz.resize(altura, vector<int>(largura));

			// Lê os pixels byte a byte
			for (int i = 0; i < altura; ++i) {
				for (int j = 0; j < largura; ++j) {
					unsigned char valorPixel;
					arquivo.read(reinterpret_cast<char*>(&valorPixel), 1);

					// Determina se o pixel é branco ou não
					matriz[i][j] = (valorPixel < 250) ? 1 : 0;
				}
			}

			arquivo.close();
			return matriz;
		}

		Point getIndex(double X, double Y){
			int linha = -(Y + this->OffsetY)/this->escala;
			int coluna = (X - this->OffsetX)/this->escala;
			return Point(linha, coluna);
		}
	
		geometry_msgs::msg::Point* getPos(int x, int y){
			geometry_msgs::msg::Point pos;
			pos.x = this->OffsetX + this->escala*x;
			pos.y = -this->OffsetY - this->escala*y;
			return pos;
		}

		vector<Point*> AStar(vector<vector<int>>& mapa, Point* inicio, Point* objetivo) {
			
			if (!dentroDosLimites(inicio->x, inicio->y, mapa.size(), mapa[0].size())){
				cout << "Ponto inicial ffora do mapa!" << endl;
				return {};
			}else if (!dentroDosLimites(objetivo->x, objetivo->y, mapa.size(), mapa[0].size())){
				cout << "Ponto Final fora do mapa!" << endl;
				return {};
			}else if (mapa[inicio->x][inicio->y] == 1){
				cout << "Ponto inicial na parede!" << endl;
				return {};
			}else if (mapa[objetivo->x][objetivo->y] == 1){
				cout << "Ponto Final na parede!" << endl;
				return {};
			}

			int direcoes[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

			auto comp = [](Point* a, Point* b) { return a->f > b->f; };
			priority_queue<Point*, vector<Point*>, decltype(comp)> filaPrioridade(comp);

			inicio->g = 0.0;
			inicio->calcularHeuristica(objetivo);
			inicio->f = inicio->g + inicio->h;
			filaPrioridade.push(inicio);

			while (!filaPrioridade.empty()) {
				Point* atual = filaPrioridade.top();
				filaPrioridade.pop();

				vector<Point*> caminho;
				if (atual->x == objetivo->x && atual->y == objetivo->y) {
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
							vizinho->g = atual->g + 1; 
							vizinho->calcularHeuristica(objetivo);
							vizinho->f = vizinho->h + vizinho->g;
							filaPrioridade.push(vizinho);
						}
						mapa[novoY][novoX] = -1;
					}
				}
			}
			return {};
		}

	public:
		CalculatePath(const std::string& name)
			: BT::StatefulActionNode(name, {}) {}

		BT::NodeStatus onStart() override {
			std::cout << "Calculating path..." << std::endl;

			return BT::NodeStatus::RUNNING;
		}

		BT::NodeStatus onRunning() override {

			this->inicio = this->getIndex(this->robotX, this->robotY);
			this->objetivo = this->getIndex(this->targetX, this->targetY);

			this->mapa = this->GetMap(this->nomeArquivo);
			
			this->caminho = this->AStar(this->mapa, this->inicio, this->objetivo);

			// aplicar getPos finalpath

			for (auto ponto : caminho){
				this->finalpath.push_back(this->getPos(ponto->x, ponto->y));
			}

			// Caminho salvo em "finalpath"

			return BT::NodeStatus::SUCCESS;
			
		}

};
