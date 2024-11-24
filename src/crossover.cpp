#include "structs.cpp"

// CROSSOVER PAPER

Nodo seleccionarClienteComun(const vector<Nodo>& parent1, const vector<Nodo>& parent2) {
    vector<Nodo> clientesComunes;

    for (const auto& nodo1 : parent1) {
        if (nodo1.tipo == 'c' && find(parent2.begin(), parent2.end(), nodo1) != parent2.end()) {
            clientesComunes.push_back(nodo1);
        }
    }

    int indiceAleatorio = rand() % clientesComunes.size();
    return clientesComunes[indiceAleatorio];
}

vector<Nodo> extraerSubruta(const vector<Nodo>& parent, const Nodo& clienteComun) {
    vector<Nodo> subruta;
    bool clienteEncontrado = false;
    int cont = 0;

    // Buscar el depósito antes del cliente común para iniciar la subruta
    for (size_t i = 0; i < parent.size(); ++i) {
        const Nodo& nodo = parent[i];

        if (nodo.tipo == 'c' && nodo == clienteComun) {
            clienteEncontrado = true;
        }

        if (nodo.tipo == 'd'){
            cont++;
            if (cont == 2 && clienteEncontrado) {
                break;
            } else {
                subruta.clear();
                cont = 1;
            }
        }

        if (nodo.tipo != 'd'){
            subruta.push_back(nodo);
        }
    }

    return subruta;
}

void imprimirRuta(const vector<Nodo>& ruta) {
    for (const Nodo& nodo : ruta) {
        if (nodo.tipo == 'd') cout << "d0 ";
        else if (nodo.tipo == 'c') cout << "c" << nodo.id << " ";
        else if (nodo.tipo == 'f') cout << "f" << nodo.id << " ";
    }
    cout << endl;
}

vector<Nodo> crearSub2(const vector<Nodo>& V1, const vector<Nodo>& V2) {
    set<Nodo> unionV1V2(V2.begin(), V2.end());
    vector<Nodo> sub2;

    for (const auto& nodo : unionV1V2) {
        if (nodo.tipo == 'c' && find(V1.begin(), V1.end(), nodo) == V1.end()) {
            sub2.push_back(nodo);
        }
    }

    return sub2;
}

vector<Nodo> concatenate(const vector<Nodo>& vec1, const vector<Nodo>& vec2) {
    vector<Nodo> result = vec1;

    if (!result.empty() && !vec2.empty() && result.back().tipo == 'd' && vec2.front().tipo == 'd') {
        result.insert(result.end(), vec2.begin() + 1, vec2.end());
    } else {
        result.insert(result.end(), vec2.begin(), vec2.end());
    }

    return result;
}

vector<Nodo> reverse(const vector<Nodo>& vec) {
    vector<Nodo> reversedVec = vec;
    reverse(reversedVec.begin(), reversedVec.end());
    return reversedVec;
}

vector<Nodo> crearHijoConReemplazo(const vector<Nodo>& parent, vector<Nodo> SC) {
    vector<Nodo> hijo;
    map<int, Nodo> nodosSC;
    for (const auto& nodo : SC) {
        if (nodo.tipo == 'c') {
            nodosSC[nodo.id] = nodo;
        }
    }

    for (const Nodo& nodo : parent) {
        if (nodo.tipo == 'c' && nodosSC.find(nodo.id) != nodosSC.end()) {
            hijo.push_back(nodosSC[nodo.id]);
            nodosSC.erase(nodo.id);
        } else {
            hijo.push_back(nodo);
        }
    }

    return hijo;
}


pair<vector<Nodo>, vector<Nodo>> crossover(const vector<Nodo>& parent1, const vector<Nodo>& parent2) {
    vector<Nodo> hijo1, hijo2, sub1, subruta2, sub2, SC1, SC2;

    Nodo clienteComun = seleccionarClienteComun(parent1, parent2);

    sub1 = extraerSubruta(parent1, clienteComun);
    subruta2 = extraerSubruta(parent2, clienteComun);
    sub2 = crearSub2(sub1, subruta2);

    SC1 = concatenate(sub2, sub1);
    SC2 = concatenate(reverse(sub1), reverse(sub2));

    hijo1 = crearHijoConReemplazo(parent1, SC1);
    hijo2 = crearHijoConReemplazo(parent2, SC2);

    return {hijo1, hijo2};
}