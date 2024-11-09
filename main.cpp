#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <cstdlib>

using namespace std;
using namespace chrono;

typedef pair<char, int> nodeKey;

struct Nodo {
    int id;
    char tipo;
    double longitud;
    double latitud;
};

struct RutaVehiculo{
	vector<nodeKey> route;
	double vehicleAcumTime = 0;
	double vehicleSolQuality = 0;
	int vehicleClients = 0;
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
    double capacidadCombustible;
    vector<Nodo> nodosClientes;
    vector<Nodo> nodosEstaciones;
    Nodo deposito;
    vector<int> clientesVisitados;
};

struct Ruta {
    vector<nodeKey> nodosVisitados;
    double distanciaTotal = 0;
    double tiempoTotal = 0;
    double combustibleRestante;
};


// Función para calcular la distancia Haversine entre dos nodos
double calcularDistanciaHaversine(double longitudActual, double latitudActual, double longitudNodoNext, double latitudNodoNext) {
    double toRadian = M_PI / 180.0;
    double radioTierra = 4182.44949;
    double latitudDeposito = (latitudNodoNext - latitudActual) * toRadian;
    double longitudDeposito = (longitudNodoNext - longitudActual) * toRadian;
    latitudActual *= toRadian;
    latitudNodoNext *= toRadian;

    double insideRootValue = sin(latitudDeposito / 2) * sin(latitudDeposito / 2) + cos(latitudActual) * cos(latitudNodoNext) * sin(longitudDeposito / 2) * sin(longitudDeposito / 2);
    return 2 * radioTierra * asin(sqrt(insideRootValue));
}



// Leer la instancia desde un archivo
Instancia* leerInstancia(const string &nombreArchivo) {
    Instancia* instancia = new Instancia();
    ifstream archivo(nombreArchivo);
    if (!archivo.is_open()) {
        cerr << "Error al abrir el archivo: " << nombreArchivo << endl;
        exit(1);
    }

    string linea;
    if (getline(archivo, linea)) {
        istringstream stream(linea);
        stream >> instancia->nombre >> instancia->numClientes >> instancia->numEstaciones 
               >> instancia->tiempoMaximo >> instancia->distanciaMaxima 
               >> instancia->velocidad >> instancia->tiempoServicio >> instancia->tiempoRecarga
               >> instancia->capacidadCombustible;
    }
 
    while (getline(archivo, linea)) {
        Nodo nodo;
        istringstream stream(linea);
        stream >> nodo.id >> nodo.tipo >> nodo.longitud >> nodo.latitud;
        if (nodo.tipo == 'd') instancia->deposito = nodo;
        else if (nodo.tipo == 'c') instancia->nodosClientes.push_back(nodo);
        else if (nodo.tipo == 'f') instancia->nodosEstaciones.push_back(nodo);
    }

    archivo.close();
    return instancia;
}

// Busca una ruta desde el nodo actual hacia el depot
// 1) Puede ser una ruta directa nodo->depot
// 2) Puede ser una ruta con AFS intermediario: nodo->AFS->depot
void verificarRegreso(double acumDist, double acumTime, double longitudActual, double latitudActual, const Instancia* instancia,
					  int& AFsID, double& timeToAFS, double& distanciaAFS, double& distanciaAFsDeposito) {
    double distanciaMinimaEncontrada = 9999999;
	double extraTime = 0;
	double dist1, dist2;
	double longitudNodoNext, latitudNodoNext, lon3, lat3;
	double subrouteDistance = 0;
	double subrouteTime = 0;
	// Longitude & Latitude of the depot node
	longitudNodoNext = instancia->deposito.longitud;
	latitudNodoNext = instancia->deposito.latitud;

	AFsID = -1;  // Inicialización indicando que no se encontró una estación de servicio válida
    timeToAFS = 0;
    distanciaAFS = 0;
    distanciaAFsDeposito = 0;

	Nodo nodoAuxiliar;
	for(int i = 0; i < instancia->numEstaciones; i++) {
		nodoAuxiliar = instancia->nodosEstaciones[i];
		subrouteDistance = 0;
		subrouteTime = 0;
		lon3 = nodoAuxiliar.longitud;
		lat3 = nodoAuxiliar.latitud;
		// Distancia entre el nodo actual y el AFS
		dist1 = calcularDistanciaHaversine(longitudActual, latitudActual, lon3, lat3);
		// Se puede viajar al AFS?
		if(dist1 + acumDist <= instancia->distanciaMaxima) {
			subrouteTime = dist1/instancia->velocidad + instancia->tiempoServicio;
			// Hay suficiente tiempo para viajar y terminar la ruta?
			if(acumTime + subrouteTime < instancia->tiempoMaximo) {
				// Distance between chosen AFS and depot
				dist2 = calcularDistanciaHaversine(lon3, lat3, longitudNodoNext, latitudNodoNext);
				// Can  travel to the depot?
				if(dist2 <= instancia->distanciaMaxima) {
					// Just enough time to travel and finish the route?
					subrouteTime += dist2/instancia->velocidad;
					if(acumTime + subrouteTime <= instancia->tiempoMaximo) {
						subrouteDistance = dist1 + dist2;
						if(subrouteDistance < distanciaMinimaEncontrada) {
							distanciaMinimaEncontrada = subrouteDistance; // minimum cost found
							// {AFS_Id, Time}, {Distance to the AFS, Distance from AFS to Depot}
							AFsID = nodoAuxiliar.id;
							timeToAFS = subrouteTime;
							distanciaAFS = dist1;
							distanciaAFsDeposito = dist2;
						}
					}
				}
			}
		}
	}
}

RutaVehiculo crearSolucionInicial(const Instancia* instancia, vector<int>& nodosClientesVisitados) {
    vector<nodeKey> rutaVehiculoGreedy;
	Nodo nodoActual = instancia->deposito;
	Nodo nodoSiguiente, nodoAuxiliar;

	nodeKey curKey = {nodoActual.tipo, nodoActual.id};
	rutaVehiculoGreedy.push_back(curKey);

	double calidadRuta = 0, tiempoEnRuta = 0;
	double distanciaProximoCliente = 0, distanciaVehiculoAcumulada = 0, distanciaAlDeposito = 0;
	double distanciaMinimaEncontrada = 9999999;
	double latitudActual, longitudActual, latitudNodoNext, longitudNodoNext, longitudDeposito, latitudDeposito;
	double auxTime = 0, nodoActualTime = 0;
	double finalReturnTime = 0, finalReturnDist = 0;
	int flagRefuel = 1, flagTerminate = 1, puedeRetornarDeposito = 0;
	int totalClientes = 0;

	// Longitud y latitud del deposito
	longitudDeposito = nodoActual.longitud;
	latitudDeposito = nodoActual.latitud;

	double timeToAFS, distanciaAFS, distanciaAFsDeposito; // tiempo a la estacion de servicio, distancia a la estacion de servicio, distancia de la estacion de servicio al deposito
	int AFsID; // AFS id para regresar al deposito

	while(tiempoEnRuta < instancia->tiempoMaximo) {

		// Recorrer todos los nodos clientes hasta encontrar el mas cercano
		for(int i = 0; i < instancia->numClientes; i++) {
			nodoAuxiliar = instancia->nodosClientes[i];

			// Si no se ha visitado el nodo cliente
			if(nodosClientesVisitados[i] == 0) {

				// Calcular distancia entre el nodo actual y el siguiente nodo cliente posible a visitar
				distanciaProximoCliente = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, nodoAuxiliar.longitud, nodoAuxiliar.latitud);	

				// Se puede viajar al nodo cliente desde el nodo actual?
				if(distanciaProximoCliente < distanciaMinimaEncontrada && distanciaVehiculoAcumulada+distanciaProximoCliente < instancia->distanciaMaxima) {

					// Distancia al deposito
					distanciaAlDeposito = calcularDistanciaHaversine(longitudDeposito, latitudDeposito, nodoAuxiliar.longitud, nodoAuxiliar.latitud);
					verificarRegreso(distanciaVehiculoAcumulada + distanciaProximoCliente, tiempoEnRuta, nodoAuxiliar.longitud, nodoAuxiliar.latitud, instancia, AFsID, timeToAFS, distanciaAFS, distanciaAFsDeposito);
					
					// Verificar si se puede regresar desde nodo actual a deposito
					if (AFsID != -1) {
						puedeRetornarDeposito = 1;

						// Se puede regresar directamente al deposito
						if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
							auxTime = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
							if (auxTime + tiempoEnRuta + distanciaAlDeposito / instancia->velocidad <= instancia->tiempoMaximo) {
								nodoSiguiente = nodoAuxiliar;
								distanciaMinimaEncontrada = distanciaProximoCliente;
								nodoActualTime = auxTime;
								flagRefuel = 0;
								flagTerminate = 0;
								finalReturnDist = distanciaAlDeposito;
								finalReturnTime = distanciaAlDeposito / instancia->velocidad;
							}
						}

						// Verificar si se regresa al deposito por la ruta nodo -> AFS -> deposito
						else if (distanciaAFS + distanciaAFsDeposito < distanciaAlDeposito && distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAFS <= instancia->distanciaMaxima) {
							auxTime = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
							if (auxTime + tiempoEnRuta + timeToAFS <= instancia->tiempoMaximo) {
								nodoSiguiente = nodoAuxiliar;
								distanciaMinimaEncontrada = distanciaProximoCliente;
								nodoActualTime = auxTime;
								flagRefuel = 0;
								flagTerminate = 0;
								finalReturnTime = timeToAFS;
								finalReturnDist = distanciaAFS + distanciaAFsDeposito;
							}
						}
					}
				}
			}
		}

		// Si no encuentra algun cliente donde viajar, intenta buscar una estacion de servicio (solo si el ultimo nodo visitado NO fue una estacion de servicio)
		if(flagRefuel && rutaVehiculoGreedy.back().first != 'f') {
			verificarRegreso(distanciaVehiculoAcumulada, tiempoEnRuta, nodoActual.longitud, nodoActual.latitud, instancia, AFsID, timeToAFS, distanciaAFS, distanciaAFsDeposito);
			if(AFsID != -1) {
				puedeRetornarDeposito = 1;
				nodoSiguiente = instancia->nodosEstaciones[AFsID];
				distanciaMinimaEncontrada = distanciaAFS;
				nodoActualTime = instancia->tiempoRecarga + distanciaAFS / instancia->velocidad;
				flagTerminate = 0;
				distanciaVehiculoAcumulada = 0;
				if(AFsID != 0) {
					finalReturnTime = distanciaAFsDeposito/instancia->velocidad;
					finalReturnDist = distanciaAFsDeposito;
				}
				else {
					finalReturnTime = distanciaAFS/instancia->velocidad;
					finalReturnDist = distanciaAFS;
				}
			}
		}
		if(flagTerminate) break;
		if(!puedeRetornarDeposito) break;

		if(!flagRefuel) {
			nodosClientesVisitados[nodoSiguiente.id-1] = 1;
		}
		tiempoEnRuta += nodoActualTime;
		distanciaVehiculoAcumulada += distanciaMinimaEncontrada;
		calidadRuta += distanciaMinimaEncontrada;
		distanciaVehiculoAcumulada += distanciaMinimaEncontrada;
		distanciaMinimaEncontrada = 9999999;
		flagRefuel = 1;
		flagTerminate = 1;
		puedeRetornarDeposito = 0;
		nodoActual = nodoSiguiente;
		if(nodoSiguiente.tipo == 'c') totalClientes++;
		rutaVehiculoGreedy.push_back({nodoSiguiente.tipo, nodoSiguiente.id});
	}
	// When last visited node is f0, dont count in the refuel time
	// exchange f0 with d0
	if(rutaVehiculoGreedy.back().first == 'f' && rutaVehiculoGreedy.back().second == 0) {
		rutaVehiculoGreedy.pop_back();
		tiempoEnRuta -= instancia->tiempoRecarga;
		finalReturnTime = 0;
		finalReturnDist = 0;
	}
	rutaVehiculoGreedy.push_back({'d', 0});
	tiempoEnRuta += finalReturnTime;
	calidadRuta += finalReturnDist;
	// Set the solution
	RutaVehiculo solution;
	solution.route = rutaVehiculoGreedy;
	solution.vehicleAcumTime = tiempoEnRuta;
	solution.vehicleSolQuality = calidadRuta;
	solution.vehicleClients = totalClientes;
	return solution;
}

void guardarSoluciones(const Instancia* instancia, const vector<RutaVehiculo>& soluciones, const string& nombreArchivo, double tiempoEjecucion) {
    ofstream archivoSalida(nombreArchivo);
    if (!archivoSalida.is_open()) {
        cerr << "Error al crear el archivo de salida: " << nombreArchivo << endl;
        return;
    }

    archivoSalida << "Calidad total\t#Clientes atendidos\t#Vehículos\tTiempo de ejecución [s]" << endl;
    double calidadTotal = 0;
    int totalClientes = 0;
    for (const auto& sol : soluciones) {
        calidadTotal += sol.vehicleSolQuality;
        totalClientes += sol.vehicleClients;
    }
    archivoSalida << calidadTotal << "\t" << totalClientes << "\t" << soluciones.size() << "\t" << tiempoEjecucion << endl << endl;

    // Guardar cada solución individualmente
    for (size_t i = 0; i < soluciones.size(); ++i) {
        const RutaVehiculo& sol = soluciones[i];
        archivoSalida << "Ruta camión #" << i + 1 << "\t";

        for (size_t j = 0; j < sol.route.size(); ++j) {
            char tipo = sol.route[j].first;
            int id = sol.route[j].second;
            if (tipo == 'd') archivoSalida << "do";
            else if (tipo == 'c') archivoSalida << "c" << id;
            else if (tipo == 'f') archivoSalida << "f" << id;
            if (j < sol.route.size() - 1) archivoSalida << "-";
        }
        archivoSalida << "\t";

        archivoSalida << "Distancia recorrida: " << sol.vehicleSolQuality << "\t";
        archivoSalida << "Tiempo transcurrido: " << sol.vehicleAcumTime << "\t";
        archivoSalida << "Distancia excedida: " << (sol.vehicleSolQuality > instancia->distanciaMaxima ? "Sí" : "No") << endl;
    }

    archivoSalida.close();
}

int main() {
    srand(static_cast<unsigned>(time(0)));

    Instancia* instancia = leerInstancia("instancias/AB101.dat");
    vector<int> nodosClientesVisitados;
    for(int i = 0; i < instancia->numClientes; i++) {
		nodosClientesVisitados.push_back(0);
	}

    vector<RutaVehiculo> soluciones;  // Vector para almacenar todas las soluciones

    auto inicio = high_resolution_clock::now();
    
    while (true) {
        RutaVehiculo sol = crearSolucionInicial(instancia, nodosClientesVisitados);
        if (sol.vehicleClients == 0)  // Terminar si una solución tiene 0 clientes
            break;

        soluciones.push_back(sol);  // Guardar solución en el vector
    }

    auto fin = high_resolution_clock::now();
    double tiempoEjecucion = duration<double>(fin - inicio).count();

    string nombreArchivoSalida = instancia->nombre + ".out";
    guardarSoluciones(instancia, soluciones, nombreArchivoSalida, tiempoEjecucion);

    return 0;
}