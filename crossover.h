#pragma once
#include <vector>
#include <set>
#include "structs.h"

// Declaraciones de funciones

Nodo seleccionarClienteComun(const std::vector<Nodo>& parent1, const std::vector<Nodo>& parent2);
std::vector<Nodo> extraerSubruta(const std::vector<Nodo>& parent, const Nodo& clienteComun);
void imprimirRuta(const std::vector<Nodo>& ruta);
std::vector<Nodo> crearSub2(const std::vector<Nodo>& V1, const std::vector<Nodo>& V2);
std::vector<Nodo> concatenate(const std::vector<Nodo>& vec1, const std::vector<Nodo>& vec2);
std::vector<Nodo> reverse(const std::vector<Nodo>& vec);
std::vector<Nodo> crearHijoConReemplazo(const std::vector<Nodo>& parent, std::vector<Nodo> SC);
std::pair<std::vector<Nodo>, std::vector<Nodo>> crossover(const std::vector<Nodo>& parent1, const std::vector<Nodo>& parent2);
