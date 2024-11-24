#include <fstream>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <iomanip> 
#include <filesystem>
#include "structs.cpp"
#include "crossover.cpp"
#include "mutacion.cpp"

using namespace chrono;

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
	double distanciaMinimaEncontrada = 99999999999;
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
		if (distanciaProximoCliente + distanciaVehiculoAcumulada < instancia->distanciaMaxima) {
			distanciaAlDeposito = calcularDistanciaHaversine(instancia->deposito.longitud, instancia->deposito.latitud, nodoClienteRandom.longitud, nodoClienteRandom.latitud);
			if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAlDeposito <= instancia->distanciaMaxima) {
				tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
				if (tiempoAux + tiempoVehiculoAcumulado + (distanciaAlDeposito / instancia->velocidad) <= instancia->tiempoMaximo) {
					tiempoFinalRegreso = distanciaAlDeposito / instancia->velocidad;
					distanciaFinalRegreso =  distanciaAlDeposito;
					distanciaVehiculoAcumulada += distanciaProximoCliente;
					tiempoVehiculoAcumulado += tiempoAux;
					calidadRuta +=  distanciaProximoCliente;
					rutaVehiculoGreedy.push_back(nodoClienteRandom);
					nodosClientesVisitados[primerClienteRandom] = 1;
					nodoActual = nodoClienteRandom;
                    totalClientes++;
				}
			} else {
				AFsID = verificarRegreso(distanciaVehiculoAcumulada + distanciaProximoCliente, tiempoVehiculoAcumulado, nodoClienteRandom.longitud, nodoClienteRandom.latitud, instancia, tiempoAFS, distanciaAFS, distanciaAFsDeposito);
				if (AFsID != -1){
					if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAFS  <= instancia->distanciaMaxima) {
						tiempoAux = (distanciaProximoCliente / instancia->velocidad) + instancia->tiempoServicio;
						if (tiempoAux + tiempoVehiculoAcumulado + tiempoAFS <= instancia->tiempoMaximo) {
							tiempoFinalRegreso = tiempoAFS;
							distanciaFinalRegreso = distanciaAFS + distanciaAFsDeposito;
							distanciaVehiculoAcumulada += distanciaProximoCliente;
							tiempoVehiculoAcumulado += tiempoAux;
							calidadRuta += distanciaProximoCliente;
							rutaVehiculoGreedy.push_back(nodoClienteRandom);
							nodosClientesVisitados[primerClienteRandom] = 1;
							nodoActual = nodoClienteRandom;
                            totalClientes++;
						}
					}
				} else {
					// busca otro cliente random
					return RutaVehiculo({}, 0, 0, -1);
				}
			}
		} else {
			// busca otro cliente random
			return RutaVehiculo({}, 0, 0, -1);
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

							if (distanciaVehiculoAcumulada + distanciaProximoCliente + distanciaAFS  <= instancia->distanciaMaxima) {
								
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

// Funcion para separar una lista concatenada de rutas de los vehiculos en un vector de vectores
vector<RutaVehiculo> separarRuta(const vector<Nodo>& rutaConcatenada, const Instancia* instancia) {
    vector<RutaVehiculo> solucion;
    vector<Nodo> rutaVehiculo;
    Nodo deposito = instancia->deposito;

    // Recorrer la ruta concatenada
    for (const auto& nodo : rutaConcatenada) {
        if (nodo.tipo == 'd') {
            if (!rutaVehiculo.empty()) {
                // Iniciar los cálculos de calidad y tiempo para esta subruta
                double calidadRuta = 0.0;
                double tiempoAcumuladoVehiculo = 0.0;
                int clientesVisitados = 0;

                // Insertar depósito al inicio y al final de la subruta
                rutaVehiculo.insert(rutaVehiculo.begin(), deposito);
                rutaVehiculo.push_back(deposito);

                Nodo nodoActual = deposito;

                // Recorrer los nodos de la subruta y calcular métricas
                for (const auto& nodoSiguiente : rutaVehiculo) {
                    double distancia = calcularDistanciaHaversine(
                        nodoActual.longitud, nodoActual.latitud,
                        nodoSiguiente.longitud, nodoSiguiente.latitud
                    );

                    calidadRuta += distancia;
                    tiempoAcumuladoVehiculo += distancia / instancia->velocidad;

                    // Verificar el tipo de nodo para agregar tiempos adicionales
                    if (nodoSiguiente.tipo == 'c') {
                        tiempoAcumuladoVehiculo += instancia->tiempoServicio;
                        clientesVisitados++;
                    } else if (nodoSiguiente.tipo == 'f') {
                        tiempoAcumuladoVehiculo += instancia->tiempoRecarga;
                    }

                    nodoActual = nodoSiguiente;
                }

                // Crear la RutaVehiculo con las métricas calculadas
                solucion.emplace_back(rutaVehiculo, tiempoAcumuladoVehiculo, calidadRuta, clientesVisitados);

                // Limpiar la subruta para la siguiente iteración
                rutaVehiculo.clear();
            }
        } else {
            rutaVehiculo.push_back(nodo);
        }
    }

    return solucion;
}


pair<double,double> verificarRutaValida(const vector<Nodo>& rutaConcatenada, const Instancia* instancia) {

    vector<RutaVehiculo> rutasSeparadas = separarRuta(rutaConcatenada, instancia);
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
        for (size_t i = 0; i < rutaVehiculo.ruta.size(); ++i) {
            Nodo nodoSiguiente = rutaVehiculo.ruta[i];
            double distancia = 0.0;

            if (nodoSiguiente.tipo == 'c') {
                Nodo cliente = instancia->nodosClientes[nodoSiguiente.id - 1];
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, cliente.longitud, cliente.latitud);
                tiempoAcumulado += (distancia / instancia->velocidad) + instancia->tiempoServicio;
				distanciaAcumulada += distancia;

                if (tiempoAcumulado > instancia->tiempoMaximo || distanciaAcumulada > instancia->distanciaMaxima) {
                    return make_pair(-1,-1);
                }

                calidadTotal += distancia;
                clientesVisitados++;

            } else if (nodoSiguiente.tipo == 'f') {
                Nodo estacion = instancia->nodosEstaciones[nodoSiguiente.id];
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, estacion.longitud, estacion.latitud);
                tiempoAcumulado += (distancia / instancia->velocidad) + instancia->tiempoRecarga;
                distanciaAcumulada += distancia;

                if (tiempoAcumulado > instancia->tiempoMaximo || distanciaAcumulada > instancia->distanciaMaxima) {
                    return make_pair(-1,-1);
                }
                
                calidadTotal += distancia;
                distanciaAcumulada = 0;

            } else if (nodoSiguiente.tipo == 'd') {
                distancia = calcularDistanciaHaversine(nodoActual.longitud, nodoActual.latitud, instancia->deposito.longitud, instancia->deposito.latitud);
                tiempoAcumulado += distancia / instancia->velocidad;
                distanciaAcumulada += distancia;

                if (distanciaAcumulada > instancia->distanciaMaxima || tiempoAcumulado > instancia->tiempoMaximo) {
                    return make_pair(-1,-1);		
                }

                calidadTotal += distancia;
            }

            nodoActual = nodoSiguiente;
        }
    }

    return make_pair(calidadTotal, clientesVisitados);
}

void guardarSoluciones(const Instancia* instancia, const vector<RutaVehiculo>& soluciones, const string& nombreArchivo, const time_point<high_resolution_clock>& inicio, int mIteraciones) {
    // Asegurarse de que el directorio 'outputs' existe
    string carpetaSalida = "resultados/outputs_" + to_string(mIteraciones) + "/";
    std::filesystem::create_directories(carpetaSalida);

    // Generar la ruta completa del archivo de salida
    string rutaArchivo = carpetaSalida + nombreArchivo;

    ofstream archivoSalida(rutaArchivo);

    auto fin = high_resolution_clock::now();
    auto tiempoTotal = duration_cast<milliseconds>(fin - inicio).count();
    double calidadTotal = 0;
    int totalClientes = 0;
    for (const auto& sol : soluciones) {
        calidadTotal += sol.calidadRuta;
        totalClientes += sol.clientesVisitados;
    }

    archivoSalida << fixed << setprecision(2);
    archivoSalida << calidadTotal << "\t" << totalClientes << "\t" << soluciones.size() << "\t" << tiempoTotal << endl << endl;

    // Guardar cada solución individualmente con alineación adecuada
    for (const auto& sol : soluciones) {
        ostringstream rutaStream;

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
        archivoSalida << left << setw(65) << rutaStream.str()
                      << right << setw(15) << sol.calidadRuta
                      << setw(15) << sol.tiempoAcumuladoVehiculo
                      << setw(10) << 0 << endl;
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


vector<vector<RutaVehiculo>> generarSolucionesIniciales(const Instancia* instancia, int cantidadPoblacion) {
	vector<vector<RutaVehiculo>> solucionesIniciales;
	for (int i = 0; i < cantidadPoblacion; i ++){
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

		vector<RutaVehiculo> solucion;
		while (true) {
			RutaVehiculo sol = crearSolucionInicial(instancia, nodosClientesVisitados, primerClienteRandom);
            if (sol.clientesVisitados == 0){
				break;
            }

            if (sol.clientesVisitados == -1) {
                primerClienteRandomIndex = rand() % listaClientes.size();
                primerClienteRandom = listaClientes[primerClienteRandomIndex];
                listaClientes.erase(listaClientes.begin() + primerClienteRandomIndex);
                //cout<<"Nuevo cliente random: "<<primerClienteRandom<<endl;
            } else {
                primerClienteRandom = -1;
			    solucion.push_back(sol);
            }
		}
		solucionesIniciales.push_back(solucion);

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

int contarClientes(const vector<Nodo>& cromosoma) {
    int contadorClientes = 0;

    for (const auto& nodo : cromosoma) {
        if (nodo.tipo == 'c') {
            contadorClientes++;
        }
    }

    return contadorClientes;
}

void algoritmoEvolutivo(int cantidadPoblacion, int mIteraciones, Instancia* instancia, const time_point<high_resolution_clock>& inicio)  {

    vector<vector<RutaVehiculo>> poblacion = generarSolucionesIniciales(instancia, cantidadPoblacion);

    // Mostrar la mejor solución inicial
    double mejorCalidadInicial = numeric_limits<double>::max();
    vector<RutaVehiculo> mejorSolucionInicial, sol1, sol2;
    int cantClientes = 0;

    for (const auto& solucion : poblacion) {
        double calidad = funcionEvaluacion(solucion);
        if (calidad < mejorCalidadInicial) {
            mejorCalidadInicial = calidad;
            mejorSolucionInicial = solucion;
            cantClientes = contarClientes(concatenarRuta(mejorSolucionInicial));
        }
    }


    cout << "Instancia: " << instancia->nombre << " Clientes: " << cantClientes <<  " Mejor calidad de la poblacion inicial: " << mejorCalidadInicial << endl;

    // Guardar en el archivo 'mejor_inicial.txt'
    ofstream archivoSalida("soluciones_greedy_" + to_string(mIteraciones) + ".txt", ios::app);
    if (archivoSalida.is_open()) {
        archivoSalida << instancia->nombre << " " << fixed << setprecision(2) << mejorCalidadInicial << endl;
        archivoSalida.close();
    } else {
        cerr << "Error al abrir el archivo 'soluciones_greedy.txt'" << endl;
    }

    vector<RutaVehiculo> mejorSolucion = mejorSolucionInicial;

	// Iterar el algoritmo evolutivo m veces
    for (int iteracion = 0; iteracion < mIteraciones; ++iteracion) {
        vector<vector<RutaVehiculo>> nuevaPoblacion;
        // Generar n hijos mejores que los padres

        while (static_cast<int>(nuevaPoblacion.size()) < cantidadPoblacion) {

            int indiceSolucion1 = rand() % cantidadPoblacion;
            int indiceSolucion2 = rand() % cantidadPoblacion;

            vector<RutaVehiculo> solucion1 = poblacion[indiceSolucion1];
            vector<RutaVehiculo> solucion2 = poblacion[indiceSolucion2];

            vector<Nodo> rutaConcatenada1 = concatenarRuta(solucion1);
            vector<Nodo> rutaConcatenada2 = concatenarRuta(solucion2);

            auto [hijo1, hijo2] = crossover(rutaConcatenada1, rutaConcatenada2);

            // Aplicar mutaciones a ambos hijos
            hijo1 = mutacion2Opt(hijo1);
            hijo2 = mutacion2Opt(hijo2);

            hijo1 = mutacionHeuristicSwap(hijo1);
            hijo2 = mutacionHeuristicSwap(hijo2);

            auto resultado1 = verificarRutaValida(hijo1, instancia);
            auto resultado2 = verificarRutaValida(hijo2, instancia);


            if (resultado1.first != -1 && resultado1.first < funcionEvaluacion(solucion1) && resultado1.first < funcionEvaluacion(solucion2)) {
                sol1 = separarRuta(hijo1, instancia);
                nuevaPoblacion.push_back(sol1);
            }

            if (resultado2.first != -1 && resultado2.first < funcionEvaluacion(solucion1) && resultado2.first < funcionEvaluacion(solucion2)) {
                sol2 = separarRuta(hijo2, instancia);
                nuevaPoblacion.push_back(sol2);
            }
        }

        poblacion = nuevaPoblacion;

        // Encontrar la mejor solucion de la nueva poblacion
        double mejorCalidad = numeric_limits<double>::max();

        for (const auto& solucion : poblacion) {
            double calidad = funcionEvaluacion(solucion);
            if (calidad < mejorCalidad) {
                mejorCalidad = calidad;
                if (funcionEvaluacion(solucion) < funcionEvaluacion(mejorSolucion)) {
                    mejorSolucion = solucion;
                }
            }
        }
    }
    guardarSoluciones(instancia, mejorSolucion, instancia->nombre + ".out", inicio, mIteraciones);
}



// ---------------------------- MAIN ----------------------------

int main() {
    srand(time(nullptr));

    vector<string> nombreInstancias = {
        "instancias/AB101.dat", "instancias/AB102.dat", "instancias/AB103.dat",
        "instancias/AB104.dat", "instancias/AB105.dat", "instancias/AB106.dat",
        "instancias/AB107.dat", "instancias/AB108.dat", "instancias/AB109.dat",
        "instancias/AB110.dat", "instancias/AB111.dat", "instancias/AB112.dat",
        "instancias/AB113.dat", "instancias/AB114.dat", "instancias/AB115.dat",
        "instancias/AB116.dat", "instancias/AB117.dat", "instancias/AB118.dat",
        "instancias/AB119.dat", "instancias/AB120.dat", 
        "instancias/AB201.dat", "instancias/AB202.dat", "instancias/AB203.dat",
        "instancias/AB204.dat", "instancias/AB205.dat", "instancias/AB206.dat",
        "instancias/AB207.dat", "instancias/AB208.dat", "instancias/AB209.dat",
        "instancias/AB210.dat", "instancias/AB211.dat", "instancias/AB212.dat",
        "instancias/AB213.dat", "instancias/AB214.dat", "instancias/AB215.dat",
        "instancias/AB216.dat", "instancias/AB217.dat", "instancias/AB218.dat",
        "instancias/AB219.dat", "instancias/AB220.dat"
    };


    vector<int> valoresIteraciones = {
        //20, 
        //25, 
        30}; 

    for (int mIteraciones : valoresIteraciones) {
        for (const string& instanciaPath : nombreInstancias) {
            Instancia* instancia = leerInstancia(instanciaPath);
            int cantidadPoblacion = instancia->numClientes - 5;

            auto inicio = high_resolution_clock::now();

            algoritmoEvolutivo(cantidadPoblacion, mIteraciones, instancia, inicio);

            delete instancia;
        }
    }

    return 0;
}