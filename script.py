import os
import re
import pandas as pd

def leer_soluciones_greedy(ruta_base, iteraciones):
    """Leer el archivo soluciones_greedy para un número específico de iteraciones."""
    archivo = os.path.join(ruta_base, f"resultados/soluciones_greedy_{iteraciones}.txt")
    soluciones_greedy = {}
    with open(archivo, "r") as file:
        for linea in file:
            if linea.strip():
                instancia, mejor_calidad = linea.split()
                soluciones_greedy[instancia] = float(mejor_calidad)
    return soluciones_greedy

def leer_output(ruta_base, instancia, iteraciones):
    """Leer el archivo output de una instancia específica para un número de iteraciones."""
    archivo = os.path.join(ruta_base, f"resultados/outputs_{iteraciones}/{instancia}.out")
    mejor_solucion = None
    tiempo_total = 0
    with open(archivo, "r") as file:
        for idx, linea in enumerate(file):
            if idx == 0:  # Primera línea contiene la distancia y el tiempo
                datos = linea.strip().split()
                mejor_solucion = float(datos[0])  # Primer valor de la línea
                tiempo_total = float(datos[3]) / 1000   # Cuarto valor de la línea
                break
    return mejor_solucion, tiempo_total

def comparar_resultados(instancias, ruta_base, iteraciones_list):
    """Comparar resultados de greedy y evolutivo para múltiples iteraciones."""
    resultados = []
    for iteraciones in iteraciones_list:
        soluciones_greedy = leer_soluciones_greedy(ruta_base, iteraciones)
        for instancia in instancias:
            mejor_greedy = soluciones_greedy.get(instancia, float("inf"))
            mejor_evolutivo, tiempo_total = leer_output(ruta_base, instancia, iteraciones)
            diferencia = ((mejor_evolutivo - mejor_greedy) / mejor_greedy) * 100
            resultados.append({
                "Instancia": instancia,
                "Iteraciones": iteraciones,
                "Mejor Greedy": mejor_greedy,
                "Mejor Evolutivo": mejor_evolutivo,
                "Diferencia (%)": diferencia,
                "Tiempo ejecución (s)": tiempo_total
            })
    return resultados

def guardar_resultados_csv(resultados, archivo_salida="resultados_comparacion.csv"):
    """Guardar los resultados en un archivo CSV."""
    df = pd.DataFrame(resultados)
    df.to_csv(archivo_salida, index=False)
    print(f"Resultados guardados en {archivo_salida}")


# Configuración
ruta_base = "./"  # Cambia esta ruta si los archivos están en otro directorio
instancias = [
    "AB101", "AB102", "AB103", "AB104", "AB105", "AB106",
    "AB107", "AB108", "AB109", "AB110", "AB111", "AB112",
    "AB113", "AB114", "AB115", "AB116", "AB117", "AB118",
    "AB119", "AB120",
    "AB201", "AB202", "AB203", "AB204", "AB205", "AB206",
    "AB207", "AB208", "AB209", "AB210", "AB211", "AB212",
    "AB213", "AB214", "AB215", "AB216", "AB217", "AB218",
    "AB219", "AB220",
]
iteraciones_list = [20, 25, 30]

# Ejecución
resultados = comparar_resultados(instancias, ruta_base, iteraciones_list)
guardar_resultados_csv(resultados)
