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
	double vehicletiempoVehiculoAcumulado = 0;
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
int verificarRegreso(double distanciaVehiculoAcumulada, double tiempoVehiculoAcumulado, double longitudActual, double latitudActual, const Instancia* instancia,
					double& tiempoAFS, double& distanciaAFS, double& distanciaAFsDeposito) {
	Nodo nodoAuxiliar;
    double distanciaMinimaEncontrada = 9999999;
	double distanciaAux1, distanciaAux2;
	double longitudDeposito, latitudDeposito;
	double distanciaAuxTotal = 0;
	double tiempoAux = 0;
	// Longitude & Latitude of the depot node
	longitudDeposito = instancia->deposito.longitud;
	latitudDeposito = instancia->deposito.latitud;

	int AFsID = -1;  // Inicialización indicando que no se encontró una estación de servicio válida
    tiempoAFS = 0;
    distanciaAFS = 0;
    distanciaAFsDeposito = 0;

	for(int i = 0; i < instancia->numEstaciones; i++) {
		nodoAuxiliar = instancia->nodosEstaciones[i];
		distanciaAuxTotal = 0;
		tiempoAux = 0;

		// Distancia entre el nodo actual y el AFS
		distanciaAux1 = calcularDistanciaHaversine(longitudActual, latitudActual, nodoAuxiliar.longitud, nodoAuxiliar.latitud);

		// If se puede viajar al AFS con el combustible restante
		if(distanciaAux1 + distanciaVehiculoAcumulada <= instancia->distanciaMaxima) {

			// Tiempo para llegar al AFS + tiempo de servicio en el AFS
			tiempoAux = distanciaAux1/instancia->velocidad + instancia->tiempoServicio;

			// Tiempo acumulado + tiempo para llegar al AFS + tiempo de servicio en el AFS <= tiempo maximo
			if(tiempoVehiculoAcumulado + tiempoAux < instancia->tiempoMaximo) {

				// Distancia entre el AFS y el deposito
				distanciaAux2 = calcularDistanciaHaversine(nodoAuxiliar.longitud, nodoAuxiliar.latitud, longitudDeposito, latitudDeposito);

				// If se puede regresar al deposito desde el AFS
				if(distanciaAux2 <= instancia->distanciaMaxima) {
					
					// Tiempo para regresar al deposito
					tiempoAux += distanciaAux2/instancia->velocidad;

					// Tiempo acumulado + tiempo para llegar al AFS + tiempo de servicio en el AFS + tiempo para regresar al deposito <= tiempo maximo
					if(tiempoVehiculoAcumulado + tiempoAux <= instancia->tiempoMaximo) {

						// Calcular la distancia total de la ruta
						distanciaAuxTotal = distanciaAux1 + distanciaAux2;

						// If la distancia total de la ruta es menor a la distancia minima encontrada
						if(distanciaAuxTotal < distanciaMinimaEncontrada) {
							distanciaMinimaEncontrada = distanciaAuxTotal; 
							AFsID = nodoAuxiliar.id;
							tiempoAFS = tiempoAux;
							distanciaAFS = distanciaAux1;
							distanciaAFsDeposito = distanciaAux2;
						}
					}
				}
			}
		}
	}
	return AFsID;
}

RutaVehiculo crearSolucionInicial(const Instancia* instancia, vector<int>& nodosClientesVisitados) {
    vector<nodeKey> rutaVehiculoGreedy;
	Nodo nodoActual = instancia->deposito;
	Nodo nodoSiguiente, nodoAuxiliar;

	nodeKey curKey = {nodoActual.tipo, nodoActual.id};
	rutaVehiculoGreedy.push_back(curKey);

	double calidadRuta = 0;
	double tiempoVehiculoAcumulado = 0;
	double distanciaProximoCliente = 0;
	double distanciaVehiculoAcumulada = 0;
	double distanciaAlDeposito = 0;
	double distanciaMinimaEncontrada = 9999999;
	double latitudActual, longitudActual;
	double latitudNodoNext, longitudNodoNext;
	double tiempoAFS; // tiempo a la estacion de servicio para regresar
	double distanciaAFS; // distancia a la estacion de servicio para regresar
	double distanciaAFsDeposito; // distancia de la estacion de servicio al deposito para regresar
	double auxTime = 0, tiempoSiguienteNodo = 0;
	double finalReturnTime = 0, finalReturnDist = 0;
	int necesitaRepostar = 1, terminarCiclo = 1, puedeRetornarDeposito = 0;
	int totalClientes = 0;
	int AFsID; // ID de la estacion de servicio para regresar al deposito

	// Longitud y latitud del deposito
	double longitudDeposito = nodoActual.longitud;
	double latitudDeposito = nodoActual.latitud;

	while(tiempoVehiculoAcumulado < instancia->tiempoMaximo) {

		// Recorrer todos los nodos clientes hasta encontrar el mas cercano
		for(int i = 0; i < instancia->numClientes; i++) {
			nodoAuxiliar = instancia->nodosClientes[i];

			// Si no se ha visitado el nodo cliente
			if(nodosClientesVisitados[i] == 0) {

				// Calcular distancia entre el nodo actual y el siguiente nodo cliente posible a visitar
				distanciaProximoCliente = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, nodoAuxiliar.longitud, nodoAuxiliar.latitud);	

				// If se encuentra un nodo mas cercano y la distancia acumulada no excede la distancia maxima del vehiculo
				if(distanciaProximoCliente < distanciaMinimaEncontrada && distanciaVehiculoAcumulada + distanciaProximoCliente < instancia->distanciaMaxima) {

					// Distancia al deposito
					distanciaAlDeposito = calcularDistanciaHaversine(longitudDeposito, latitudDeposito, nodoAuxiliar.longitud, nodoAuxiliar.latitud);

					// Verificar si se puede regresar al deposito desde el nodo actual
					AFsID = verificarRegreso(distanciaVehiculoAcumulada + distanciaProximoCliente, tiempoVehiculoAcumulado, nodoAuxiliar.longitud, nodoAuxiliar.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);
					
					// Verificar si se puede regresar desde nodo actual a deposito
					if (AFsID != -1) {
						puedeRetornarDeposito = 1;

						// Se puede regresar directamente al deposito
						if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
							
							// Tiempo para llegar a nodo cliente + tiempo de servicio de atender al cliente
							auxTime = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;

							// auxTime + tiempo acumulado del vehiculo + tiempo para regresar al deposito <= tiempo maximo
							if (auxTime + tiempoVehiculoAcumulado + (distanciaAlDeposito / instancia->velocidad) <= instancia->tiempoMaximo) {
								nodoSiguiente = nodoAuxiliar;
								distanciaMinimaEncontrada = distanciaProximoCliente;
								tiempoSiguienteNodo = auxTime;
								necesitaRepostar = 0;
								terminarCiclo = 0;
								finalReturnDist = distanciaAlDeposito;
								finalReturnTime = distanciaAlDeposito / instancia->velocidad;
							}
						}

						// Verificar si se regresa al deposito por la ruta nodo -> AFS -> deposito
						else if (distanciaAFS + distanciaAFsDeposito < distanciaAlDeposito && distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAFS <= instancia->distanciaMaxima) {
							
							// Tiempo para llegar a nodo cliente + tiempo de servicio de atender al cliente
							auxTime = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;

							// auxTime + tiempo acumulado del vehiculo + tiempo para regresar al deposito <= tiempo maximo
							if (auxTime + tiempoVehiculoAcumulado + tiempoAFS <= instancia->tiempoMaximo) {
								nodoSiguiente = nodoAuxiliar;
								distanciaMinimaEncontrada = distanciaProximoCliente;
								tiempoSiguienteNodo = auxTime;
								necesitaRepostar = 0;
								terminarCiclo = 0;
								finalReturnTime = tiempoAFS;
								finalReturnDist = distanciaAFS + distanciaAFsDeposito;
							}
						}
					}
				}
			}
		}

		// Si no encuentra algun cliente donde viajar, intenta buscar una estacion de servicio (si el ultimo nodo visitado no fue una estacion de servicio)
		if(necesitaRepostar && rutaVehiculoGreedy.back().first != 'f') {
			AFsID = verificarRegreso(distanciaVehiculoAcumulada, tiempoVehiculoAcumulado, nodoActual.longitud, nodoActual.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);
			if(AFsID != -1) {
				puedeRetornarDeposito = 1;
				nodoSiguiente = instancia->nodosEstaciones[AFsID];
				distanciaMinimaEncontrada = distanciaAFS;
				tiempoSiguienteNodo = instancia->tiempoRecarga + (distanciaAFS / instancia->velocidad);
				terminarCiclo = 0;
				distanciaVehiculoAcumulada = 0;

				// Si es distinto de
				if(AFsID != 0) {
					finalReturnTime = distanciaAFsDeposito/instancia->velocidad;
					finalReturnDist = distanciaAFsDeposito;
				}

				// Si la estacion de servicio es el deposito
				else {
					finalReturnTime = distanciaAFS/instancia->velocidad;
					finalReturnDist = distanciaAFS;
				}
			}
		}
		if(terminarCiclo || !puedeRetornarDeposito) break;

		if(!necesitaRepostar) 
			nodosClientesVisitados[nodoSiguiente.id-1] = 1;
		
		distanciaVehiculoAcumulada += distanciaMinimaEncontrada;
		tiempoVehiculoAcumulado += tiempoSiguienteNodo;
		distanciaVehiculoAcumulada += distanciaMinimaEncontrada;
		calidadRuta += distanciaMinimaEncontrada;
		distanciaMinimaEncontrada = 9999999;
		necesitaRepostar = 1;
		terminarCiclo = 1;
		puedeRetornarDeposito = 0;
		nodoActual = nodoSiguiente;
		if(nodoSiguiente.tipo == 'c') 
			totalClientes++;
		rutaVehiculoGreedy.push_back({nodoSiguiente.tipo, nodoSiguiente.id});
	}
	
	// Si el ultimo nodo visitado fue una estacion de servicio, se elimina de la ruta 
	if(rutaVehiculoGreedy.back().first == 'f' && rutaVehiculoGreedy.back().second == 0) {
		rutaVehiculoGreedy.pop_back();
		tiempoVehiculoAcumulado -= instancia->tiempoRecarga;
		finalReturnTime = 0;
		finalReturnDist = 0;
	}
	rutaVehiculoGreedy.push_back({'d', 0});
	tiempoVehiculoAcumulado += finalReturnTime;
	calidadRuta += finalReturnDist;
	
	RutaVehiculo solution;
	solution.route = rutaVehiculoGreedy;
	solution.vehicletiempoVehiculoAcumulado = tiempoVehiculoAcumulado;
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
        archivoSalida << "Tiempo transcurrido: " << sol.vehicletiempoVehiculoAcumulado << "\t";
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