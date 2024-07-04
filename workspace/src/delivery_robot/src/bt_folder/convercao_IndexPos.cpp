#include <iostream>
#include<tuple>
#include <vector>


using namespace std;

struct Point {
	int x, y;
	double f, g, h; 
	Point* parent; 
	Point(int x, int y) : x(x), y(y), f(0), g(0), h(0), parent(nullptr) {}
	double calcularHeuristica(Point* goal) {
		// h = sqrt(pow((x - goal->x), 2) + pow((y - goal->y), 2));	
		h = x - goal->x + y - goal->y;
		return h;
	}
};


Point toIndex(double X, double Y){ // converte a posição real para indice na matriz
	float escala = 0.03;
	float OffsetX = -15.1;
	float OffsetY = -25.0;	
	int linha = -(Y + OffsetY)/escala;
	int coluna = (X - OffsetX)/escala;
	return Point(coluna, linha);
}

tuple <double,double> toPos(int x, int y){
	float escala = 0.03;
	float OffsetX = -15.1;
	float OffsetY = -25.0;	
	double X, Y;
	
	X = OffsetX + escala*x;
	Y = -OffsetY - escala*y;
	tuple <double,double> posicao = make_tuple(X,Y);
	return posicao;
}

int main(){
	double x = -5.1;
	double y = 17.0;
	cout << "Pos: x " << x << " y " << y << endl;
	Point saida = toIndex(x,y);
	cout << "toIndex: x " << saida.x << " y " << saida.y <<  endl;
	
	tuple <double,double> s = toPos(saida.x, saida.y);
	cout << "toPos: x "<< get<0>(s) << " y " << get<1>(s) << endl;
	 
	
	
    
    
    
    return 0;

}
