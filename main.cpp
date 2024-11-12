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
#include <algorithm>

#include "structs.h"
#include "crossover.h"

using namespace std;
using namespace chrono;

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

	for(int i = 1; i < instancia->numEstaciones; i++) {
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

	double tiempoRegresoDeposito = 0;
	double calidadRuta = 0;
	double distanciaProximoCliente = 0;
	double distanciaVehiculoAcumulada = 0;
	double tiempoVehiculoAcumulado = 0;
	double distanciaAlDeposito = 0;
	double distanciaMinimaEncontrada = 9999999;
	double latitudActual, longitudActual;
	double latitudNodoNext, longitudNodoNext;
	double tiempoAFS; // tiempo hasta AFS + AFS->deposito
	double distanciaAFS; // distancia a la estacion de servicio para regresar
	double distanciaAFsDeposito; // distancia de la estacion de servicio al deposito para regresar
	double tiempoAux = 0, tiempoSiguienteNodo = 0;
	double tiempoFinalRegreso = 0, distanciaFinalRegreso = 0;
	int necesitaRepostar = 1, terminarEjecucion = 1, puedeRetornarDeposito = 0;
	int totalClientes = 0;
	int AFsID; // ID de la estacion de servicio para regresar al deposito

	// Longitud y latitud del deposito
	double longitudDeposito = nodoActual.longitud;
	double latitudDeposito = nodoActual.latitud;

	// Verifica si el primer cliente random cumple restricciones de tiempo y distancia, si no, elige otro
	if (primerClienteRandom != -1) {
		Nodo nodoClienteRandom = instancia->nodosClientes[primerClienteRandom];
		distanciaProximoCliente = calcularDistanciaHaversine(instancia->deposito.longitud, instancia->deposito.latitud, nodoClienteRandom.longitud, nodoClienteRandom.latitud);
		if (distanciaProximoCliente < instancia->distanciaMaxima) {
			distanciaAlDeposito = calcularDistanciaHaversine(instancia->deposito.longitud, instancia->deposito.latitud, nodoClienteRandom.longitud, nodoClienteRandom.latitud);
			if (distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
				tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
				if (tiempoAux + (distanciaAlDeposito / instancia->velocidad) <= instancia->tiempoMaximo) {
					tiempoFinalRegreso = distanciaAlDeposito / instancia->velocidad;
					distanciaFinalRegreso =  distanciaAlDeposito;
					distanciaVehiculoAcumulada += distanciaProximoCliente;
					tiempoVehiculoAcumulado += tiempoAux;
					calidadRuta +=  distanciaProximoCliente;
					rutaVehiculoGreedy.push_back(nodoClienteRandom);
					nodosClientesVisitados[primerClienteRandom] = 1;
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

				// If se encuentra un nodo mas cercano y la distancia acumulada no excede la distancia maxima del vehiculo
				if(distanciaProximoCliente < distanciaMinimaEncontrada && distanciaVehiculoAcumulada + distanciaProximoCliente < instancia->distanciaMaxima) {

					// Se debe verificar si se puede regresar al deposito desde el nodo actual + al que se piensa mover
					distanciaAlDeposito = calcularDistanciaHaversine(longitudDeposito, latitudDeposito, nodoAuxiliar.longitud, nodoAuxiliar.latitud);

					// Verificar si se puede regresar al deposito desde el nodo actual + al que se piensa mover
					if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
						puedeRetornarDeposito = 1;

						// Tiempo para llegar a nodo cliente + tiempo de servicio de atender al cliente
						tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
						tiempoRegresoDeposito = distanciaAlDeposito / instancia->velocidad;

						// tiempoAux + tiempo acumulado del vehiculo + tiempo para regresar al deposito <= tiempo maximo
						if (tiempoAux + tiempoVehiculoAcumulado + tiempoRegresoDeposito <= instancia->tiempoMaximo) {
							nodoSiguiente = nodoAuxiliar;
							distanciaMinimaEncontrada = distanciaProximoCliente;
							tiempoSiguienteNodo = tiempoAux;
							necesitaRepostar = 0;
							terminarEjecucion = 0;
							distanciaFinalRegreso = distanciaAlDeposito;
							tiempoFinalRegreso = tiempoRegresoDeposito;
						}
					}
					
					// Si no se puede regresar directamente al deposito, intenta buscar una estacion de servicio para devolverse
					else {
						AFsID = verificarRegreso(distanciaVehiculoAcumulada + distanciaProximoCliente, tiempoVehiculoAcumulado, nodoAuxiliar.longitud, nodoAuxiliar.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);

						if (AFsID != -1) {
							puedeRetornarDeposito = 1;

							if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAFS + distanciaAFsDeposito <= instancia->distanciaMaxima) {
								
								tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;

								if (tiempoAux + tiempoVehiculoAcumulado + tiempoAFS <= instancia->tiempoMaximo) {
									nodoSiguiente = nodoAuxiliar;
									distanciaMinimaEncontrada = distanciaProximoCliente;
									tiempoSiguienteNodo = tiempoAux;
									necesitaRepostar = 0;
									terminarEjecucion = 0;
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
			
			if(AFsID != -1) {
				puedeRetornarDeposito = 1;
				nodoSiguiente = instancia->nodosEstaciones[AFsID];
				distanciaMinimaEncontrada = distanciaAFS;
				tiempoSiguienteNodo = instancia->tiempoRecarga + (distanciaAFS / instancia->velocidad);
				terminarEjecucion = 0;
				distanciaVehiculoAcumulada = 0;
				tiempoFinalRegreso = distanciaAFsDeposito/instancia->velocidad;
				distanciaFinalRegreso = distanciaAFsDeposito;
			}
		}

		if(terminarEjecucion || !puedeRetornarDeposito){
			break;
		}

		if(!necesitaRepostar) 
			nodosClientesVisitados[nodoSiguiente.id-1] = 1;
		
		distanciaVehiculoAcumulada += distanciaMinimaEncontrada;
		tiempoVehiculoAcumulado += tiempoSiguienteNodo;
		calidadRuta += distanciaMinimaEncontrada;
		distanciaMinimaEncontrada = 9999999;
		necesitaRepostar = 1;
		terminarEjecucion = 1;
		puedeRetornarDeposito = 0;
		nodoActual = nodoSiguiente;
		if(nodoSiguiente.tipo == 'c') 
			totalClientes++;
		rutaVehiculoGreedy.push_back(nodoSiguiente);
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
                      << std::setw(10) << 0 << endl;
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


pair<double,double> verificarRutaValida(const vector<Nodo>& rutaConcatenada, const Instancia* instancia) {
    vector<RutaVehiculo> rutasSeparadas = separarRuta(rutaConcatenada);
	double calidadTotal = 0;
	double clientesVisitados = 0;

    for (const auto& rutaVehiculo : rutasSeparadas) {
        double distanciaAcumulada = 0.0;
        double tiempoAcumulado = 0.0;
        Nodo nodoActual = instancia->deposito;

        if (rutaVehiculo.ruta.front().tipo != 'd' || rutaVehiculo.ruta.back().tipo != 'd') {
            return make_pair(-1,-1);
        }

        // Recorrer la ruta del vehículo
        for (size_t i = 1; i < rutaVehiculo.ruta.size(); ++i) {
            Nodo nodoSiguiente = rutaVehiculo.ruta[i];
            double distancia = 0.0;

            if (nodoSiguiente.tipo == 'c') {
                Nodo cliente = instancia->nodosClientes[nodoSiguiente.id - 1];
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, cliente.longitud, cliente.latitud);
                tiempoAcumulado += (distancia / instancia->velocidad) + instancia->tiempoServicio;
				clientesVisitados++;
            } else if (nodoSiguiente.tipo == 'f') {
                Nodo estacion = instancia->nodosEstaciones[nodoSiguiente.id];
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, estacion.longitud, estacion.latitud);
                tiempoAcumulado += (distancia / instancia->velocidad) + instancia->tiempoRecarga;
                distanciaAcumulada = 0; 
            } else if (nodoSiguiente.tipo == 'd') {
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, instancia->deposito.longitud, instancia->deposito.latitud);
            }

            distanciaAcumulada += distancia;
			calidadTotal += distancia;

            if (distanciaAcumulada > instancia->distanciaMaxima || tiempoAcumulado > instancia->tiempoMaximo) {
                return make_pair(-1,-1);		
            }

            nodoActual = nodoSiguiente;
        }
    }

    return make_pair(calidadTotal, clientesVisitados);
}

vector<vector<RutaVehiculo>> generarSolucionesIniciales(const Instancia* instancia, int cantidadPoblacionInicial) {
	vector<vector<RutaVehiculo>> solucionesIniciales;
	for (int i = 0; i < cantidadPoblacionInicial; i ++){
		vector<int> nodosClientesVisitados(instancia->numClientes, 0);
        vector<int> listaClientes;

        // Inicializar la lista de clientes
        for (int i = 0; i < instancia->numClientes; i++) {
            listaClientes.push_back(i);
        }

        // Seleccionar el primer cliente aleatorio y removerlo de la lista
        int primerClienteRandomIndex = rand() % listaClientes.size();
        int primerClienteRandom = listaClientes[primerClienteRandomIndex];
        listaClientes.erase(listaClientes.begin() + primerClienteRandomIndex);
        nodosClientesVisitados[primerClienteRandom] = 1;

		vector<RutaVehiculo> solucion;
		while (true) {
			RutaVehiculo sol = crearSolucionInicial(instancia, nodosClientesVisitados, primerClienteRandom);
			if (sol.clientesVisitados == 0)  
				break;

			// Verificar si se encontró una solución válida
            if (sol.ruta.size() == 0 && !listaClientes.empty()) {
                // Seleccionar un nuevo cliente aleatorio de la lista y removerlo
                primerClienteRandomIndex = rand() % listaClientes.size();
                primerClienteRandom = listaClientes[primerClienteRandomIndex];
                listaClientes.erase(listaClientes.begin() + primerClienteRandomIndex);
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

    	//guardarSoluciones(instancia, solucion, instancia->nombre + "_" + to_string(i+1) + ".out");
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

void algoritmoEvolutivo(vector<vector<RutaVehiculo>> poblacionInicial, int cantidadPoblacionInicial, Instancia* instancia) {
    bool flag = true;
	bool flag2 = false;

	while (flag == true) {
		// selecciona 2 soluciones aleatorias y realiza el cruzamiento
		int indiceSolucion1 = rand() % cantidadPoblacionInicial;
		int indiceSolucion2 = rand() % cantidadPoblacionInicial;

		vector<RutaVehiculo> solucion1 = poblacionInicial[indiceSolucion1];
		vector<RutaVehiculo> solucion2 = poblacionInicial[indiceSolucion2];

		vector<Nodo> rutaConcatenada1 = concatenarRuta(solucion1);
		vector<Nodo> rutaConcatenada2 = concatenarRuta(solucion2);

		auto [hijo1, hijo2] = crossover(rutaConcatenada1, rutaConcatenada2);

		auto resultado1 =  verificarRutaValida(hijo1, instancia);
		auto resultado2 = verificarRutaValida(hijo2, instancia);

		if (resultado1.first != -1 && resultado1.first < funcionEvaluacion(solucion1) && resultado1.first < funcionEvaluacion(solucion2)) {
			cout << "Ruta 1 valida con calidad: " << resultado1.first << " clientes visitados: " << resultado1.second << endl;
			flag2 = true;
		}

		if (resultado2.first != -1 && resultado2.first < funcionEvaluacion(solucion1) && resultado2.first < funcionEvaluacion(solucion2)) {
			cout << "Ruta 2 valida con calidad: " << resultado2.first << " clientes visitados: " << resultado2.second << endl;
			flag2 = true;
		}


		if (flag2){
			cout << "Calidad padres: " << funcionEvaluacion(solucion1) << " y " << funcionEvaluacion(solucion2) << endl;
			flag = false;
		} 
	
	}
}

// ---------------------------- MAIN ----------------------------

int main() {
	srand(time(nullptr));

    vector<vector<RutaVehiculo>> solucionesIniciales;  // Vector para almacenar todas las soluciones
    Instancia* instancia = leerInstancia("instancias/AB101.dat");
	int cantidadPoblacionInicial = instancia->numClientes;

    auto inicio = high_resolution_clock::now();

	solucionesIniciales = generarSolucionesIniciales(instancia, cantidadPoblacionInicial);

	algoritmoEvolutivo(solucionesIniciales, cantidadPoblacionInicial, instancia);

    auto fin = high_resolution_clock::now();
    double tiempoEjecucion = duration<double>(fin - inicio).count();
	cout << "tiempo ejecucion: " << tiempoEjecucion << endl;

    return 0;
}