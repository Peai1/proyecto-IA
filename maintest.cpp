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
typedef std::pair<std::pair<int, double>, std::pair<double, double>> AFSDepotRouteInfo;

struct Nodo {
    int id;
    char tipo;
    double longitud;
    double latitud;
};

class vehicleSolution {
public:
    double vehicleAcumTime = 0;
    double vehicleSolQuality = 0;
    int vehicleClients = 0;
    vector<nodeKey> route;

    void setVehicleSolution(vector<nodeKey> r, double time, double quality, int clients) {
        route = r;
        vehicleAcumTime = time;
        vehicleSolQuality = quality;
        vehicleClients = clients;
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
    double capacidadCombustible;
    vector<Nodo> nodosClientes;
    vector<Nodo> nodosEstaciones;
    Nodo deposito;
};

struct Ruta {
    vector<nodeKey> nodosVisitados;
    double distanciaTotal = 0;
    double tiempoTotal = 0;
    double combustibleRestante;
};

struct Solucion {
    vector<Ruta> rutas;
    double calidad = 0;
};

// Función para calcular la distancia Haversine entre dos nodos
double calcularDistancia(double lon1, double lat1, double lon2, double lat2) {
    double toRadian = M_PI / 180.0;
    double radioTierra = 4182.44949;
    double dLat = (lat2 - lat1) * toRadian;
    double dLon = (lon2 - lon1) * toRadian;
    lat1 *= toRadian;
    lat2 *= toRadian;

    double insideRootValue = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
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

AFSDepotRouteInfo verificarRegreso(double acumDist, double acumTime, double lon1, double lat1, const Instancia* instancia) {
    double minFoundDistance = 9999999;
	double extraTime = 0;
	double dist1, dist2;
	double lon2, lon3, lat2, lat3;
	double subrouteDistance = 0;
	double subrouteTime = 0;
	AFSDepotRouteInfo solution = {{-1, 0}, {0, 0}}; // default solution
	// Longitude & Latitude of the depot node
	lon2 = instancia->deposito.longitud;
	lat2 = instancia->deposito.latitud;
	Nodo auxNode;
	for(int i = 0; i < instancia->numEstaciones; i++) {
		auxNode = instancia->nodosEstaciones[i];
		subrouteDistance = 0;
		subrouteTime = 0;
		lon3 = auxNode.longitud;
		lat3 = auxNode.latitud;
		// Distance between current node & AFS
		dist1 = calcularDistancia(lon1, lat1, lon3, lat3);
		// Can travel to this AFS?
		if(dist1 + acumDist <= instancia->distanciaMaxima) {
			// We have enough time to travel
			subrouteTime = dist1/instancia->velocidad + instancia->tiempoServicio;
			if(acumTime + subrouteTime < instancia->tiempoMaximo) {
				// Distance between chosen AFS and depot
				dist2 = calcularDistancia(lon3, lat3, lon2, lat2);
				// Can  travel to the depot?
				if(dist2 <= instancia->distanciaMaxima) {
					// Just enough time to travel and finish the route?
					subrouteTime += dist2/instancia->velocidad;
					if(acumTime + subrouteTime <= instancia->tiempoMaximo) {
						subrouteDistance = dist1 + dist2;
						if(subrouteDistance < minFoundDistance) {
							minFoundDistance = subrouteDistance; // minimum cost found
							extraTime = subrouteTime; // time it would take to travel
							solution =  {{auxNode.id, extraTime}, {dist1, dist2}};
							// {AFS_Id, Time}, {Distance to the AFS, Distance from AFS to Depot}
						}
					}
				}
			}
		}
	}
	return solution;
}

vehicleSolution crearSolucionInicial(const Instancia* instancia, vector<int> visitedCustomerNodes) {
    vector<nodeKey> greedyRoute;
	Nodo curNode = instancia->deposito;
	Nodo nextNode, auxNode;

	nodeKey curKey = {curNode.tipo, curNode.id};
	greedyRoute.push_back(curKey);

	double qualityGreedy = 0, timeGreedy = 0;
	double curDistance = 0, acumulatedDistance = 0, toDepotDist = 0;
	double minFoundDistance = 9999999;
	double lat1, lon1, lat2, lon2, dLon, dLat;
	double auxTime = 0, curNodeTime = 0;
	double finalReturnTime = 0, finalReturnDist = 0;
	int flagRefuel = 1, flagTerminate = 1, flagCanReturn = 0;
	int totalClients = 0;
	AFSDepotRouteInfo returnInfo;

	// Depot longitud and latitud
	dLon = curNode.longitud;
	dLat = curNode.latitud;

	while(timeGreedy < instancia->tiempoMaximo) {
		lon1 = curNode.longitud;
		lat1 = curNode.latitud;
		for(int i = 0; i < instancia->numClientes; i++) {
			auxNode = instancia->nodosClientes[i];
			// If node is unvisited
			if(visitedCustomerNodes[i] == 0) {
				lon2 = auxNode.longitud;
				lat2 = auxNode.latitud;
				curDistance = calcularDistancia(lon1, lat1, lon2, lat2);
				
				if(curDistance < minFoundDistance && acumulatedDistance+curDistance < instancia->distanciaMaxima) {
					// Distance between cur node and depot
					toDepotDist = calcularDistancia(dLon, dLat, lon2, lat2);
					returnInfo = verificarRegreso(acumulatedDistance + curDistance, timeGreedy, lon2, lat2, instancia);
					// Can we return from this new node to depot by some route?
					if(returnInfo.first.first != -1) {
						flagCanReturn = 1;
						if(returnInfo.second.first + returnInfo.second.second < toDepotDist) {
							if(acumulatedDistance + curDistance + returnInfo.second.first <= instancia->distanciaMaxima) {
								auxTime = (curDistance / instancia->velocidad) + instancia->tiempoServicio;
								if(auxTime + timeGreedy + returnInfo.first.second <= instancia->tiempoMaximo) {
									nextNode = auxNode;
									minFoundDistance = curDistance;
									curNodeTime = auxTime;
									flagRefuel = 0;
									flagTerminate = 0;
									finalReturnTime = returnInfo.first.second;
									finalReturnDist = returnInfo.second.first + returnInfo.second.second;
								}
							}

						}
						else {
							// Its cheaper to return to deposit directely than by AFS->depot route
							if(acumulatedDistance + curDistance + toDepotDist <= instancia->distanciaMaxima) {
								auxTime = (curDistance / instancia->velocidad) + instancia->tiempoServicio;
								if(auxTime + timeGreedy + toDepotDist / instancia->velocidad <= instancia->tiempoMaximo) {
									nextNode = auxNode;
									minFoundDistance = curDistance;
									curNodeTime = auxTime;
									flagRefuel = 0;
									flagTerminate = 0;
									finalReturnDist = toDepotDist;
									finalReturnTime = toDepotDist / instancia->velocidad;
								}
							}
						}
					}
				}
			}
		}
		// Find a refuel station if cant travel to any client 
		// and only if the last node visited WAS NOT a station
		if(flagRefuel && greedyRoute.back().first != 'f') {
			returnInfo = verificarRegreso(acumulatedDistance, timeGreedy, curNode.longitud, curNode.latitud, instancia);
			if(returnInfo.first.first != -1) {
				flagCanReturn = 1;
				nextNode = instancia->nodosEstaciones[returnInfo.first.first];
				minFoundDistance = returnInfo.second.first;
				curNodeTime = instancia->tiempoRecarga + returnInfo.second.first / instancia->velocidad;
				flagTerminate = 0;
				acumulatedDistance = 0;
				if(returnInfo.first.first != 0) {
					finalReturnTime = returnInfo.second.second/instancia->velocidad;
					finalReturnDist = returnInfo.second.second;
				}
				else {
					finalReturnTime = returnInfo.second.first/instancia->velocidad;
					finalReturnDist = returnInfo.second.first;
				}
			}
		}
		if(flagTerminate) break;
		if(!flagCanReturn) break;

		if(!flagRefuel) {
			visitedCustomerNodes[nextNode.id-1] = 1;
		}
		timeGreedy += curNodeTime;
		acumulatedDistance += minFoundDistance;
		qualityGreedy += minFoundDistance;
		acumulatedDistance += minFoundDistance;
		minFoundDistance = 9999999;
		flagRefuel = 1;
		flagTerminate = 1;
		flagCanReturn = 0;
		curNode = nextNode;
		if(nextNode.tipo == 'c') totalClients++;
		greedyRoute.push_back({nextNode.tipo, nextNode.id});
	}
	// When last visited node is f0, dont count in the refuel time
	// exchange f0 with d0
	if(greedyRoute.back().first == 'f' && greedyRoute.back().second == 0) {
		greedyRoute.pop_back();
		timeGreedy -= instancia->tiempoRecarga;
		finalReturnTime = 0;
		finalReturnDist = 0;
	}
	greedyRoute.push_back({'d', 0});
	timeGreedy += finalReturnTime;
	qualityGreedy += finalReturnDist;
	// Set the solution
	vehicleSolution solution;
	solution.setVehicleSolution(greedyRoute, timeGreedy, qualityGreedy, totalClients);
	return solution;
}

void guardarSolucion(const Instancia* instancia, const vehicleSolution &solucion, const string &nombreArchivo, double tiempoEjecucion) {
    ofstream archivoSalida(nombreArchivo);
    if (!archivoSalida.is_open()) {
        cerr << "Error al crear el archivo de salida: " << nombreArchivo << endl;
        return;
    }

    archivoSalida << "Calidad de la solución\t#Clientes atendidos\t#Vehículos\tTiempo de ejecución [s]" << endl;
    archivoSalida << solucion.vehicleSolQuality << "\t" << solucion.vehicleClients << "\t" << 1 << "\t" << tiempoEjecucion << endl << endl;

    archivoSalida << "Ruta camión #1\t";

    for (size_t j = 0; j < solucion.route.size(); ++j) {
        char tipo = solucion.route[j].first;
        int id = solucion.route[j].second;
        if (tipo == 'd') archivoSalida << "do";
        else if (tipo == 'c') archivoSalida << "c" << id;
        else if (tipo == 'f') archivoSalida << "f" << id;
        if (j < solucion.route.size() - 1) archivoSalida << "-";
    }
    archivoSalida << "\t";

    archivoSalida << "Distancia recorrida: " << solucion.vehicleSolQuality << "\t";
    archivoSalida << "Tiempo transcurrido: " << solucion.vehicleAcumTime << "\t";
    archivoSalida << "Distancia excedida: " << (solucion.vehicleSolQuality > instancia->distanciaMaxima ? "Sí" : "No") << endl;

    archivoSalida.close();
}

int main() {
    srand(static_cast<unsigned>(time(0)));
    Instancia *instancia = leerInstancia("instancias/AB101.dat");
    vector<int> visitedCustomerNodes;
    cout << "111" << endl;
    for(int i = 0; i < instancia->numClientes; i++) {
		visitedCustomerNodes.push_back(0);
	}

    cout << "1" << endl;
    auto inicio = high_resolution_clock::now();
    vehicleSolution solucionInicial = crearSolucionInicial(instancia, visitedCustomerNodes);
    cout << "2" << endl;
    auto fin = high_resolution_clock::now();
    double tiempoEjecucion = duration<double>(fin - inicio).count();

    string nombreArchivoSalida = instancia->nombre + ".out";
    guardarSolucion(instancia, solucionInicial, nombreArchivoSalida, tiempoEjecucion);

    return 0;
}