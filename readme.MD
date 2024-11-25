# Proyecto de Algoritmo Evolutivo para el GVRP

## Archivos Principales

- **`src/AE.cpp`**: Archivo principal donde se ejecuta el algoritmo evolutivo.
- **`src/structs.cpp`**: Contiene las estructuras necesarias para representar el problema.
- **`src/mutacion.cpp`**: Implementa los operadores de mutación.
- **`src/crossover.cpp`**: Contiene las funciones de *crossover* para el algoritmo evolutivo.
- **`resultados_comparacion.csv`**: Archivo .csv que contiene las comparaciones para al ejecución de todas las instancias con 20, 25 y 30 iteraciones
- **`script.py`**: Archivo python que crea el .csv mencionado en base a los resultados
- **`Carpeta resultados`**: Contiene una carpeta `outputs_<nro_iteraciones>` que contiene los outputs para ejecutar el algoritmo con el *nro_iteraciones* correspondiente. También se tiene un archivo .txt `soluciones_greedy_<nro_iteraciones>.txt` el cual contiene la mejor solucion inicial greedy para cada instancia en la cantidad de iteraciones correspondientes
## Uso del `Makefile`

1. Compilar el Proyecto
```bash
make
```

Este comando compila todos los archivos necesarios y genera el ejecutable AE.

2. Ejecutar el Proyecto
```bash
make run INSTANCIA=instancias/<nombre_instancia>.dat ITERACIONES=<numero_iteraciones>
```
Este comando compila (si es necesario) y ejecuta el programa con una instancia y un número de iteraciones específicos.

Ejemplo:

```bash
make run INSTANCIA=instancias/AB101.dat ITERACIONES=30
```

Si se ejecuta 
```bash
make run
```
Se ejecuta con estos valores por defecto: 
<nombre_instancia> = AB101
<numero_iteraciones> = 20

Ejecutar esto resulta con el output <nombre_instancia>.out con el resultado del algoritmo evolutivo.

3. Limpiar archivos

Para limpiar los archivos se puede escribir el comando
```bash
make clean
```
