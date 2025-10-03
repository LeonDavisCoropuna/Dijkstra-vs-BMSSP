import matplotlib.pyplot as plt
import sys

def read_benchmark_data(filename):
    """Lee los datos del benchmark desde un archivo"""
    algorithms = {}
    nodes = set()
    
    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                
                parts = line.split()
                if len(parts) == 3:
                    algo, n, t = parts
                    n = int(n)
                    t = float(t)
                    nodes.add(n)
                    
                    if algo not in algorithms:
                        algorithms[algo] = []
                    algorithms[algo].append((n, t))
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo '{filename}'")
        sys.exit(1)
    except Exception as e:
        print(f"Error al leer el archivo: {e}")
        sys.exit(1)
    
    return algorithms, sorted(nodes)

def plot_benchmark(algorithms, nodes, output_file=None):
    """Genera la gráfica de comparación"""
    
    # Preparar datos para graficar
    plot_data = {}
    for algo, values in algorithms.items():
        plot_data[algo] = [t for n, t in sorted(values)]
    
    # Definir colores y marcadores para cada algoritmo
    styles = {
        "DIJKSTRA": {"marker": "o", "color": "#2E86AB", "label": "Dijkstra"},
        "BMSSP": {"marker": "s", "color": "#A23B72", "label": "BMSSP"},
        "ASTAR_EUCLIDEAN": {"marker": "^", "color": "#F18F01", "label": "A* (Euclidiana)"},
        "ASTAR_MANHATTAN": {"marker": "D", "color": "#C73E1D", "label": "A* (Manhattan)"}
    }
    
    # Graficar
    plt.figure(figsize=(12, 7))
    
    for algo, times in plot_data.items():
        style = styles.get(algo, {"marker": "x", "color": "gray", "label": algo})
        plt.plot(nodes, times, 
                 marker=style["marker"], 
                 color=style["color"],
                 label=style["label"],
                 linewidth=2,
                 markersize=8)
    
    plt.xlabel("Número de nodos", fontsize=12)
    plt.ylabel("Tiempo (ms)", fontsize=12)
    plt.title("Comparación de Algoritmos de Camino Más Corto", fontsize=14, fontweight='bold')
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Gráfica guardada en: {output_file}")
    
    plt.show()

def print_summary(algorithms, nodes):
    """Imprime un resumen de los resultados"""
    print("\n" + "="*60)
    print("RESUMEN DE RESULTADOS")
    print("="*60)
    
    for n in nodes:
        print(f"\nNodos: {n}")
        print("-" * 40)
        times = {}
        for algo, values in algorithms.items():
            for node, time in values:
                if node == n:
                    times[algo] = time
                    break
        
        # Encontrar el más rápido
        if times:
            min_time = min(times.values())
            for algo, time in sorted(times.items()):
                marker = " ⭐" if time == min_time else ""
                print(f"  {algo:20s}: {time:8.3f} ms{marker}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python plot_benchmark.py <archivo_resultados> [archivo_salida.png]")
        print("\nEjemplo:")
        print("  python plot_benchmark.py resultados.txt")
        print("  python plot_benchmark.py resultados.txt grafica.png")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    # Leer datos
    algorithms, nodes = read_benchmark_data(input_file)
    
    if not algorithms:
        print("Error: No se encontraron datos válidos en el archivo")
        sys.exit(1)
    
    # Imprimir resumen
    print_summary(algorithms, nodes)
    
    # Generar gráfica
    plot_benchmark(algorithms, nodes, output_file)