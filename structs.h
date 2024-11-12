#pragma once
#include <vector>
#include <string>

using namespace std;


struct Nodo {
    int id;
    char tipo;
    double longitud;
    double latitud;


    bool operator==(const Nodo& other) const;

    bool operator<(const Nodo& other) const;
};


struct RutaVehiculo {
    vector<Nodo> ruta;
    double tiempoAcumuladoVehiculo;
    double calidadRuta;
    int clientesVisitados;


    RutaVehiculo(const vector<Nodo>& ruta, double tiempoAcumuladoVehiculo, double calidadRuta, int clientesVisitados);
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