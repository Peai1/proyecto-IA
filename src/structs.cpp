#ifndef STRUCTS
#define STRUCTS

#include <vector>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <map>
#include <iostream>

using namespace std;

struct Nodo {
    int id;
    char tipo;
    double longitud;
    double latitud;

    bool operator==(const Nodo& other) const {
        return id == other.id && tipo == other.tipo;
    }

    bool operator<(const Nodo& other) const {
        if (tipo != other.tipo) return tipo < other.tipo;
        return id < other.id;
    }
};

struct RutaVehiculo{
	vector<Nodo> ruta;
	double tiempoAcumuladoVehiculo = 0;
	double calidadRuta = 0;
	int clientesVisitados = 0;

	RutaVehiculo(const vector<Nodo>& ruta, double tiempoAcumuladoVehiculo, 
		double calidadRuta, int clientesVisitados){
		this->ruta = ruta;
		this->tiempoAcumuladoVehiculo = tiempoAcumuladoVehiculo;
		this->calidadRuta = calidadRuta;
		this->clientesVisitados = clientesVisitados;
	}
};

struct Instancia {
    string nombre;
    int numClientes;
    int numEstaciones;
    double tiempoMaximo;
    double distanciaMaxima;
    double velocidad;
    double tiempoServicio;
    double tiempoRecarga;
    vector<Nodo> nodosClientes;
    vector<Nodo> nodosEstaciones;
    Nodo deposito;
};

double calcularDistanciaHaversine(double longitudActual, double latitudActual, double longitudNodoNext, double latitudNodoNext) {
    double R = 4182.44949; 
    latitudActual = latitudActual * (M_PI / 180.0);
    longitudActual = longitudActual * (M_PI / 180.0);
    latitudNodoNext = latitudNodoNext * (M_PI / 180.0);
    longitudNodoNext = longitudNodoNext * (M_PI / 180.0);

    double dlat = latitudNodoNext - latitudActual;
    double dlon = longitudNodoNext - longitudActual;

    // Formula de Haversine
    double a = pow(sin(dlat / 2), 2) + cos(latitudActual) * cos(latitudNodoNext) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = R * c;

    return distance;
}

#endif