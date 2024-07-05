#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <fstream>
#include <algorithm>

using namespace std;

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

bool dentroDosLimites(int x, int y, int linhas, int colunas) {
    return (y >= 0 && y < linhas && x >= 0 && x < colunas);
}
bool dangerpos(vector<vector<int>>& mapa, int x, int y){
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

vector<Point*> encontrarCaminhoAStar(vector<vector<int>>& mapa, Point* inicio, Point* objetivo) {
    
    if (!dentroDosLimites(inicio->x, inicio->y, mapa.size(), mapa[0].size())){
        cout << "Ponto inicial ffora do mapa!" << endl;
        return {};
    }
    
    if (!dentroDosLimites(objetivo->x, objetivo->y, mapa.size(), mapa[0].size())){
        cout << "Ponto Final fora do mapa!" << endl;
        return {};
    }

    if (mapa[inicio->x][inicio->y] == 1){
        cout << "Ponto inicial na parede!" << endl;
        return {};
    }
    
    if (mapa[objetivo->x][objetivo->y] == 1){
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

// Função para gerar um arquivo PGM a partir da matriz com o caminho e os pontos visitados marcados
void gerarArquivoPGM(vector<vector<int>>& mapa, vector<Point*>& caminho, Point* inicio, Point*objetivo,string nomeArquivo) {
    int linhas = mapa.size();
    int colunas = mapa[0].size();

    ofstream arquivo(nomeArquivo);

    if (!arquivo.is_open()) {
        cerr << "Erro ao criar o arquivo PGM." << endl;
        return;
    }

    arquivo << "P2" << endl;
    arquivo << colunas << " " << linhas << endl;
    arquivo << "255" << endl;

    // Copia o mapa original para um mapa de intensidades
    vector<vector<int>> intensidades(linhas, vector<int>(colunas, 255));

    // Marca o caminho encontrado com cinza escuro (intensidade 100)
    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            if (mapa[i][j] == -1) {
                intensidades[i][j] = 200;
            }
            else if (mapa[i][j] == 1){
                intensidades[i][j] = 0;

            }
        }
    }    
    
    for (auto ponto : caminho) {
        intensidades[ponto->y][ponto->x] = 150;
    }
    int k = 8;
    for (int i = -k; i <= k; i++){
        for (int j = -k; j <= k; j++){
            intensidades[inicio->y+i][inicio->x+j] = 50;
            intensidades[objetivo->y+i][objetivo->x+j] = 100;
        }
    }

    // Escreve as intensidades no arquivo PGM
    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            arquivo << intensidades[i][j] << " ";
        }
        arquivo << endl;
    }

    arquivo.close();
}


vector<vector<int>> lerArquivoPGM(string nomeArquivo) {
    ifstream arquivo(nomeArquivo, ios::binary);
    vector<vector<int>> matriz;

    if (!arquivo.is_open()) {
        cerr << "Erro ao abrir o arquivo." << endl;
        return matriz; // Retorna matriz vazia se houver erro na abertura do arquivo
    }

    // Variáveis para armazenar informações do cabeçalho PGM
    string tipo;
    int largura, altura, maxValor;

    // Lê o tipo do arquivo PGM (deve ser 'P5')
    getline(arquivo, tipo);
    if (tipo != "P5") {
        cerr << "O arquivo não é um arquivo PGM no formato binário (P5)." << endl;
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
    matriz.resize(altura, vector<int>(largura));

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

int main() {

    string nomeArquivo = "my_map.pgm"; 

    vector<vector<int>> mapa = lerArquivoPGM(nomeArquivo);

    int linhas = mapa.size();
    int colunas = mapa[0].size();

    Point* inicio = new Point(colunas/2-20, linhas/2);
    Point* objetivo = new Point(60, 20);
    
    vector<Point*> caminho = encontrarCaminhoAStar(mapa, inicio, objetivo);

    gerarArquivoPGM(mapa, caminho, inicio, objetivo, "saida2.pgm");
    
    if (!caminho.empty()) {

        cout << "Caminho encontrado:" << endl;
        

    } else {
        cout << "Não foi possível encontrar um caminho." << endl;
    }

    return 0;
}
