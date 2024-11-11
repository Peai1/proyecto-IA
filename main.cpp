#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <cstdlib>
#include <iomanip> 
#include <set>

using namespace std;
using namespace chrono;

struct Nodo {
    int id;
    char tipo;
    double longitud;
    double latitud;
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

// Distancia haversine
double calcularDistanciaHaversine(double longitudActual, double latitudActual, double longitudNodoNext, double latitudNodoNext) {
    double R = 4182.44949; 
    // Convertir latitudes y longitudes a radianes
    latitudActual = latitudActual * (M_PI / 180.0);
    longitudActual = longitudActual * (M_PI / 180.0);
    latitudNodoNext = latitudNodoNext * (M_PI / 180.0);
    longitudNodoNext = longitudNodoNext * (M_PI / 180.0);

    // Diferencias de latitud y longitud
    double dlat = latitudNodoNext - latitudActual;
    double dlon = longitudNodoNext - longitudActual;

    // Formula de Haversine
    double a = pow(sin(dlat / 2), 2) + cos(latitudActual) * cos(latitudNodoNext) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = R * c;

    return distance;
}

// Leer la instancia desde un archivo
Instancia* leerInstancia(const string &nombreArchivo) {
    Instancia* instancia = new Instancia();
    ifstream archivo(nombreArchivo);

    string linea;
    if (getline(archivo, linea)) {
        istringstream stream(linea);
        stream >> instancia->nombre >> instancia->numClientes >> instancia->numEstaciones 
               >> instancia->tiempoMaximo >> instancia->distanciaMaxima 
               >> instancia->velocidad >> instancia->tiempoServicio >> instancia->tiempoRecarga;
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

// Busca una ruta desde el nodo actual hacia el deposito a traves de un AFS
int verificarRegreso(double distanciaVehiculoAcumulada, double tiempoVehiculoAcumulado, double longitudActual, double latitudActual, const Instancia* instancia,
					double& tiempoAFS, double& distanciaAFS, double& distanciaAFsDeposito) {
	Nodo nodoAuxiliar;
    double distanciaMinimaEncontrada = 9999999;
	double distanciaAux1, distanciaAux2;
	double longitudDeposito, latitudDeposito;
	double distanciaAuxTotal = 0;
	double tiempoAux = 0;
	longitudDeposito = instancia->deposito.longitud;
	latitudDeposito = instancia->deposito.latitud;

	int AFsID = -1;  // Inicializacion indicando que no se encuentra una estación de servicio para regresar
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
			tiempoAux = distanciaAux1/instancia->velocidad + instancia->tiempoRecarga;

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

RutaVehiculo crearSolucionInicial(const Instancia* instancia, vector<int>& nodosClientesVisitados, int primerClienteRandom) {
    vector<Nodo> rutaVehiculoGreedy;
	Nodo nodoSiguiente, nodoAuxiliar;
	Nodo nodoActual = instancia->deposito;

	rutaVehiculoGreedy.push_back(instancia->deposito);

	double calidadRuta = 0;
	double distanciaProximoCliente = 0;
	double distanciaVehiculoAcumulada = 0;
	double distanciaAlDeposito = 0;
	double distanciaMinimaEncontrada = 9999999;
	double latitudActual, longitudActual;
	double latitudNodoNext, longitudNodoNext;
	double tiempoAFS; // tiempo hasta AFS + AFS->deposito
	double distanciaAFS; // distancia a la estacion de servicio para regresar
	double distanciaAFsDeposito; // distancia de la estacion de servicio al deposito para regresar
	double tiempoAux = 0, tiempoSiguienteNodo = 0;
	double tiempoFinalRegreso = 0, distanciaFinalRegreso = 0;
	int necesitaRepostar = 1, terminarCiclo = 1, puedeRetornarDeposito = 0;
	int totalClientes = 0;
	int AFsID; // ID de la estacion de servicio para regresar al deposito

	// Longitud y latitud del deposito
	double longitudDeposito = nodoActual.longitud;
	double latitudDeposito = nodoActual.latitud;

	double tiempoVehiculoAcumulado = 0;

	// Verifica si el primer cliente random cumple restricciones de tiempo y distancia, si no, elige otro
	if (primerClienteRandom != -1) {
		Nodo nodoClienteRandom = instancia->nodosClientes[primerClienteRandom];
		distanciaProximoCliente = calcularDistanciaHaversine(instancia->deposito.longitud, instancia->deposito.latitud, nodoClienteRandom.longitud, nodoClienteRandom.latitud);
		if (distanciaProximoCliente < instancia->distanciaMaxima) {
			distanciaAlDeposito = calcularDistanciaHaversine(instancia->deposito.longitud, instancia->deposito.latitud, nodoClienteRandom.longitud, nodoClienteRandom.latitud);
			if (distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
				tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
				if (tiempoAux + (distanciaAlDeposito / instancia->velocidad) <= instancia->tiempoMaximo) {
					tiempoFinalRegreso = tiempoAFS;
					distanciaFinalRegreso =  distanciaAlDeposito;
					distanciaVehiculoAcumulada += distanciaProximoCliente;
					tiempoVehiculoAcumulado += tiempoAux;
					calidadRuta +=  distanciaProximoCliente;
					rutaVehiculoGreedy.push_back(nodoClienteRandom);
					nodosClientesVisitados[primerClienteRandom ] = 1;
					nodoActual = nodoClienteRandom;
				}
			} else {
				AFsID = verificarRegreso(distanciaProximoCliente, tiempoVehiculoAcumulado, nodoClienteRandom.longitud, nodoClienteRandom.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);
				if (AFsID != -1){
					if (distanciaProximoCliente + distanciaAFS + distanciaAFsDeposito <= instancia->distanciaMaxima) {
						tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
						if (tiempoAux + tiempoAFS <= instancia->tiempoMaximo) {
							tiempoFinalRegreso = tiempoAFS;
							distanciaFinalRegreso = distanciaAFS + distanciaAFsDeposito;
							distanciaVehiculoAcumulada += distanciaProximoCliente;
							tiempoVehiculoAcumulado += tiempoAux;
							calidadRuta += distanciaProximoCliente;
							rutaVehiculoGreedy.push_back(nodoClienteRandom);
							nodosClientesVisitados[primerClienteRandom] = 1;
							nodoActual = nodoClienteRandom;
						}
					}
				} else {
					// busca otro cliente random
					return RutaVehiculo({}, 0, 0, 0);
				}
			}
		} else {
			// busca otro cliente random
			return RutaVehiculo({}, 0, 0, 0);
		}
	}

	while(tiempoVehiculoAcumulado < instancia->tiempoMaximo) {

		// Recorrer todos los nodos clientes hasta encontrar el mas cercano
		for(int i = 0; i < instancia->numClientes; i++) {
			nodoAuxiliar = instancia->nodosClientes[i];

			// Si no se ha visitado el nodo cliente
			if(nodosClientesVisitados[i] == 0) {

				// Calcular distancia entre el nodo actual y el siguiente nodo cliente posible a visitar
				distanciaProximoCliente = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, nodoAuxiliar.longitud, nodoAuxiliar.latitud);	

				// If se encuentra un nodo mas cercano y la distancia para llegar no excede la distancia maxima
				if(distanciaProximoCliente < distanciaMinimaEncontrada && distanciaVehiculoAcumulada + distanciaProximoCliente < instancia->distanciaMaxima) {

					// Distancia al deposito
					distanciaAlDeposito = calcularDistanciaHaversine(longitudDeposito, latitudDeposito, nodoAuxiliar.longitud, nodoAuxiliar.latitud);

					// Verificar si se puede regresar al deposito desde el nodo actual + al que se piensa mover
					if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
						puedeRetornarDeposito = 1;

						// Tiempo para llegar a nodo cliente + tiempo de servicio de atender al cliente
						tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;

						// tiempoAux + tiempo acumulado del vehiculo + tiempo para regresar al deposito <= tiempo maximo
						if (tiempoAux + tiempoVehiculoAcumulado + (distanciaAlDeposito / instancia->velocidad) <= instancia->tiempoMaximo) {
							nodoSiguiente = nodoAuxiliar;
							distanciaMinimaEncontrada = distanciaProximoCliente;
							tiempoSiguienteNodo = tiempoAux;
							necesitaRepostar = 0;
							terminarCiclo = 0;
							distanciaFinalRegreso = distanciaAlDeposito;
							tiempoFinalRegreso = distanciaAlDeposito / instancia->velocidad;
						}
					}
					
					// Si no se puede regresar directamente al deposito, intenta buscar una estacion de servicio para devolverse
					else {
						AFsID = verificarRegreso(distanciaVehiculoAcumulada + distanciaProximoCliente, tiempoVehiculoAcumulado, nodoAuxiliar.longitud, nodoAuxiliar.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);

						if (AFsID != -1) {
							puedeRetornarDeposito = 1;

							if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAFS + distanciaAFsDeposito <= instancia->distanciaMaxima) {
								
								tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;

								if (tiempoAux + tiempoVehiculoAcumulado + tiempoAFS  <= instancia->tiempoMaximo) {
									nodoSiguiente = nodoAuxiliar;
									distanciaMinimaEncontrada = distanciaProximoCliente;
									tiempoSiguienteNodo = tiempoAux;
									necesitaRepostar = 0;
									terminarCiclo = 0;
									tiempoFinalRegreso = tiempoAFS;
									distanciaFinalRegreso = distanciaAFS + distanciaAFsDeposito;
								}
							}
						}
					}
				}
			}
		}
		// Si no encuentra algun cliente donde viajar, intenta buscar una estacion de servicio (si el ultimo nodo visitado no fue una estacion de servicio)
		if(necesitaRepostar && rutaVehiculoGreedy.back().tipo != 'f') {
			// Verificar si se puede regresar al deposito desde el nodo actual
			AFsID = verificarRegreso(distanciaVehiculoAcumulada, tiempoVehiculoAcumulado, nodoActual.longitud, nodoActual.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);
			cout << "repostar1: " << AFsID << endl;
			if(AFsID != -1) {
				cout << "repostar2: " << AFsID << endl;
				puedeRetornarDeposito = 1;
				nodoSiguiente = instancia->nodosEstaciones[AFsID];
				distanciaMinimaEncontrada = distanciaAFS;
				tiempoSiguienteNodo = instancia->tiempoRecarga + (distanciaAFS / instancia->velocidad);
				terminarCiclo = 0;
				distanciaVehiculoAcumulada = 0;

				// Si es distinto del deposito
				if(AFsID != 0) {
					tiempoFinalRegreso = distanciaAFsDeposito/instancia->velocidad;
					distanciaFinalRegreso = distanciaAFsDeposito;
				}

				// Si la estacion de servicio es el deposito
				else {
					tiempoFinalRegreso = distanciaAFS/instancia->velocidad;
					distanciaFinalRegreso = distanciaAFS;
				}
			}
		}
		if(terminarCiclo || !puedeRetornarDeposito) break;

		if(!necesitaRepostar) 
			nodosClientesVisitados[nodoSiguiente.id-1] = 1;
		
		distanciaVehiculoAcumulada += distanciaMinimaEncontrada;
		tiempoVehiculoAcumulado += tiempoSiguienteNodo;
		calidadRuta += distanciaMinimaEncontrada;
		distanciaMinimaEncontrada = 9999999;
		necesitaRepostar = 1;
		terminarCiclo = 1;
		puedeRetornarDeposito = 0;
		nodoActual = nodoSiguiente;
		if(nodoSiguiente.tipo == 'c') 
			totalClientes++;
		rutaVehiculoGreedy.push_back(nodoSiguiente);
	}
	
	// Si el ultimo nodo visitado fue una estacion de servicio y era el deposito, se elimina y se resta el tiempo de recarga
	if(rutaVehiculoGreedy.back().tipo == 'f' && rutaVehiculoGreedy.back().id == 0) {
		rutaVehiculoGreedy.pop_back();
		tiempoVehiculoAcumulado -= instancia->tiempoRecarga;
		tiempoFinalRegreso = 0;
		distanciaFinalRegreso = 0;
	}

	rutaVehiculoGreedy.push_back(instancia->deposito);
	tiempoVehiculoAcumulado += tiempoFinalRegreso;
	calidadRuta += distanciaFinalRegreso;
	
	RutaVehiculo solution(rutaVehiculoGreedy, tiempoVehiculoAcumulado, calidadRuta, totalClientes);
	return solution;
}

void guardarSoluciones(const Instancia* instancia, const vector<RutaVehiculo>& soluciones, const string& nombreArchivo, double tiempoEjecucion = 0) {
    ofstream archivoSalida(nombreArchivo);

    double calidadTotal = 0;
    int totalClientes = 0;
    for (const auto& sol : soluciones) {
        calidadTotal += sol.calidadRuta;
        totalClientes += sol.clientesVisitados;
    }

    archivoSalida << std::fixed << std::setprecision(2);
    archivoSalida << calidadTotal << "\t" << totalClientes << "\t" << soluciones.size() << "\t" << tiempoEjecucion << endl << endl;

    // Guardar cada solución individualmente con alineación adecuada
    for (const auto& sol : soluciones) {
        std::ostringstream rutaStream;

        // Generar la cadena de la ruta
        for (size_t j = 0; j < sol.ruta.size(); ++j) {
            char tipo = sol.ruta[j].tipo;
            int id = sol.ruta[j].id;
            if (tipo == 'd') rutaStream << "d0";
            else if (tipo == 'c') rutaStream << "c" << id;
            else if (tipo == 'f') rutaStream << "f" << id;
            if (j < sol.ruta.size() - 1) rutaStream << "-";
        }

        // Imprimir la ruta y los valores numéricos alineados
        archivoSalida << std::left << std::setw(65) << rutaStream.str()
                      << std::right << std::setw(15) << sol.calidadRuta
                      << std::setw(15) << sol.tiempoAcumuladoVehiculo
                      << std::setw(10) << (sol.calidadRuta > instancia->distanciaMaxima ? "Sí" : "No") << endl;
    }
}

// Funcion para concatenar una lista de rutas en un vector
vector<Nodo> concatenarRuta(const vector<RutaVehiculo>& solucion) {
    vector<Nodo> rutaConcatenada;

    for (const auto& vehiculo : solucion) {
        for (const auto& nodo : vehiculo.ruta) {
			if (nodo.tipo == 'd' && !rutaConcatenada.empty() && rutaConcatenada.back().tipo == 'd') {
                continue;
            }
            rutaConcatenada.push_back(nodo);
        }
    }
    return rutaConcatenada;
}

// Funcion para separar una lista concatenada de rutas de los vehiculos en un vector de vectores
vector<RutaVehiculo> separarRuta(const vector<Nodo>& rutaConcatenada) {
    vector<RutaVehiculo> solucion;
    vector<Nodo> rutaVehiculo;
    Nodo deposito = {0, 'd', 0.0, 0.0};

    // Recorrer la ruta concatenada
    for (const auto& nodo : rutaConcatenada) {
        if (nodo.tipo == 'd') {
            if (!rutaVehiculo.empty()) {
                rutaVehiculo.insert(rutaVehiculo.begin(), deposito);
                rutaVehiculo.push_back(deposito);
                solucion.emplace_back(rutaVehiculo, 0.0, 0.0, rutaVehiculo.size());
                rutaVehiculo.clear();
            }
        } else {
            rutaVehiculo.push_back(nodo);
        }
	}
    return solucion;
}


bool verificarRutaValida(const vector<Nodo>& rutaConcatenada, const Instancia* instancia) {
    vector<RutaVehiculo> rutasSeparadas = separarRuta(rutaConcatenada);
    Nodo deposito = {0, 'd', 0.0, 0.0};

    for (const auto& rutaVehiculo : rutasSeparadas) {
        double distanciaAcumulada = 0.0;
        double tiempoAcumulado = 0.0;
        Nodo nodoActual = deposito;

        if (rutaVehiculo.ruta.front().tipo != 'd' || rutaVehiculo.ruta.back().tipo != 'd') {
            return false;
        }

        // Recorrer la ruta del vehículo
        for (size_t i = 1; i < rutaVehiculo.ruta.size(); ++i) {
            Nodo nodoSiguiente = rutaVehiculo.ruta[i];
            double distancia = 0.0;

            if (nodoSiguiente.tipo == 'c') {
                Nodo cliente = instancia->nodosClientes[nodoSiguiente.id - 1];
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, cliente.longitud, cliente.latitud);
                tiempoAcumulado += (distancia / instancia->velocidad) + instancia->tiempoServicio;
            } else if (nodoSiguiente.tipo == 'f') {
                Nodo estacion = instancia->nodosEstaciones[nodoSiguiente.id];
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, estacion.longitud, estacion.latitud);
                tiempoAcumulado += (distancia / instancia->velocidad) + instancia->tiempoRecarga;
                distanciaAcumulada = 0; 
            } else if (nodoSiguiente.tipo == 'd') {
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, deposito.longitud, deposito.latitud);
            }

            distanciaAcumulada += distancia;

            if (distanciaAcumulada > instancia->distanciaMaxima || tiempoAcumulado > instancia->tiempoMaximo) {
                return false;
            }

            nodoActual = nodoSiguiente;
        }
    }

    return true;
}

vector<vector<RutaVehiculo>> generarSolucionesIniciales(const Instancia* instancia, int cantidadPoblacionInicial) {
	vector<vector<RutaVehiculo>> solucionesIniciales;
	for (int i = 0; i < cantidadPoblacionInicial; i ++){
		vector<int> nodosClientesVisitados;
		for(int i = 0; i < instancia->numClientes; i++) {
			nodosClientesVisitados.push_back(0);
		}

		cout << "solucion nro: " << i + 1 << endl;
        std::set<int> clientesGenerados;
        int primerClienteRandom;
        do {
            primerClienteRandom = rand() % instancia->numClientes + 1; 
        } while (clientesGenerados.count(primerClienteRandom) > 0);
        clientesGenerados.insert(primerClienteRandom);
        nodosClientesVisitados[primerClienteRandom - 1] = 1;

		vector<RutaVehiculo> solucion;
		while (true) {
			RutaVehiculo sol = crearSolucionInicial(instancia, nodosClientesVisitados, primerClienteRandom);
			if (sol.clientesVisitados == 0)  
				break;

			if (sol.ruta.size() == 0){
				// genera otro cliente random
				do {
					primerClienteRandom = rand() % instancia->numClientes + 1; 
				} while (clientesGenerados.count(primerClienteRandom) > 0);
				clientesGenerados.insert(primerClienteRandom);
				nodosClientesVisitados[primerClienteRandom] = 1;
			} else {
				primerClienteRandom = -1;
			}

			solucion.push_back(sol);
		}
		solucionesIniciales.push_back(solucion);

		// test
		vector<Nodo> ola = concatenarRuta(solucion);
		vector<RutaVehiculo> god = separarRuta(ola);

    	guardarSoluciones(instancia, solucion, instancia->nombre + "_" + to_string(i+1) + ".out");
	}
	return solucionesIniciales;
}

int funcionEvaluacion(const vector<RutaVehiculo>& solucion) {
	double calidadTotal = 0;
	for (const auto& ruta : solucion) {
		calidadTotal += ruta.calidadRuta;
	}

	return calidadTotal;
}

int main() {
	srand(time(nullptr));

    vector<vector<RutaVehiculo>> solucionesIniciales;  // Vector para almacenar todas las soluciones
    Instancia* instancia = leerInstancia("instancias/AB101.dat");
	int cantidadPoblacionInicial = 10;

    auto inicio = high_resolution_clock::now();

	solucionesIniciales = generarSolucionesIniciales(instancia, cantidadPoblacionInicial);

    auto fin = high_resolution_clock::now();
    double tiempoEjecucion = duration<double>(fin - inicio).count();

    return 0;
}