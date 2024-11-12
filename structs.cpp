#include "structs.h"

// Implementación del operador ==
bool Nodo::operator==(const Nodo& other) const {
    return id == other.id && tipo == other.tipo;
}

// Implementación del operador <
bool Nodo::operator<(const Nodo& other) const {
    if (tipo != other.tipo) return tipo < other.tipo;
    return id < other.id;
}

// Implementación del constructor de RutaVehiculo
RutaVehiculo::RutaVehiculo(const vector<Nodo>& ruta, double tiempoAcumuladoVehiculo, double calidadRuta, int clientesVisitados) {
    this->ruta = ruta;
    this->tiempoAcumuladoVehiculo = tiempoAcumuladoVehiculo;
    this->calidadRuta = calidadRuta;
    this->clientesVisitados = clientesVisitados;
}
