# Proyecto de Algoritmo Evolutivo para el GVRP

Este proyecto implementa un algoritmo evolutivo para resolver el problema de enrutamiento de vehículos ecológicos (*Green Vehicle Routing Problem*, GVRP). El proyecto incluye un sistema de compilación y ejecución a través de un `Makefile`.

## Archivos Principales

- **`src/AE.cpp`**: Archivo principal donde se ejecuta el algoritmo evolutivo.
- **`src/structs.cpp`**: Contiene las estructuras necesarias para representar el problema.
- **`src/mutacion.cpp`**: Implementa los operadores de mutación.
- **`src/crossover.cpp`**: Contiene las funciones de *crossover* para el algoritmo evolutivo.
- **`Makefile`**: Sistema de compilación para automatizar la construcción y ejecución del proyecto.

## Uso del `Makefile`

1. Compilar el Proyecto
bash
Copiar código
make
Este comando compila todos los archivos necesarios y genera el ejecutable AE.

2. Limpiar Archivos Compilados
bash
Copiar código
make clean
Elimina los archivos .o generados y el ejecutable AE.

3. Ejecutar el Proyecto
bash
Copiar código
make run INSTANCIA=instancias/<nombre_instancia>.dat ITERACIONES=<numero_iteraciones>
Este comando compila (si es necesario) y ejecuta el programa con una instancia y un número de iteraciones específicos.

Ejemplo:

bash
Copiar código
make run INSTANCIA=instancias/AB101.dat ITERACIONES=30

### Compilar el Proyecto
```bash
make