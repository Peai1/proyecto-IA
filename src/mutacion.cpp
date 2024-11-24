#ifndef MUTACION
#define MUTACION

#include "structs.cpp"
// mutacion 2-opt

vector<Nodo> mutacion2Opt(const vector<Nodo>& rutaOriginal) {
    vector<Nodo> rutaMutada = rutaOriginal;
    size_t n = rutaMutada.size();

    vector<size_t> indicesDepositos;
    for (size_t i = 0; i < n; ++i) {
        if (rutaMutada[i].tipo == 'd') {
            indicesDepositos.push_back(i);
        }
    }

    size_t start, end;
    do {
        start = rand() % n;
        end = rand() % n;
    } while (start >= end || find(indicesDepositos.begin(), indicesDepositos.end(), start) != indicesDepositos.end() ||
             find(indicesDepositos.begin(), indicesDepositos.end(), end) != indicesDepositos.end());

    reverse(rutaMutada.begin() + start, rutaMutada.begin() + end + 1);

    return rutaMutada;
}


// mutacion heuristic swap paper

vector<Nodo> mutacionHeuristicSwap(const vector<Nodo>& cromosoma) {
    vector<Nodo> cromosomaMutado = cromosoma;
    size_t n = cromosomaMutado.size();

    vector<size_t> indicesClientes;
    for (size_t i = 0; i < n; ++i) {
        if (cromosomaMutado[i].tipo == 'c') {
            indicesClientes.push_back(i);
        }
    }

    if (indicesClientes.empty()) {
        return cromosomaMutado; // No hay clientes para mutar
    }

    size_t indexCi = indicesClientes[rand() % indicesClientes.size()];
    Nodo clienteCi = cromosomaMutado[indexCi];

    double distanciaMinima = numeric_limits<double>::max();
    size_t indexCj = indexCi;

    for (size_t i = 0; i < n; ++i) {
        if (i != indexCi && cromosomaMutado[i].tipo == 'c') {
            double distancia = calcularDistanciaHaversine(clienteCi.longitud, clienteCi.latitud, cromosomaMutado[i].longitud, cromosomaMutado[i].latitud);
            if (distancia < distanciaMinima) {
                distanciaMinima = distancia;
                indexCj = i;
            }
        }
    }

    if (indexCj != indexCi) {
        swap(cromosomaMutado[indexCi], cromosomaMutado[indexCj]);
    }

    return cromosomaMutado;
}

#endif