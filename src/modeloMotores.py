import numpy as np

def calcular_modelo_cuadratico(x, y):
    """
    Calcula los coeficientes del modelo cuadrático que mejor ajusta los datos.
    
    Args:
        x (list o array): Vector de valores independientes.
        y (list o array): Vector de valores dependientes.
    
    Returns:
        tuple: Coeficientes del modelo cuadrático (a, b, c).
    """
    # Ajustar un polinomio de grado 2
    coeficientes = np.polyfit(x, y, 2)
    return coeficientes

# Ejemplo de uso
#y = [100, 90, 80, 70, 60, 50, 40, 30, 20, 10]
#x = [3285, 3690, 4200, 4895, 5810, 7320, 9670, 14060, 26700, 34547]
y = [100, 90, 80, 70, 60, 50, 40, 30]
x = [3285, 3690, 4200, 4895, 5810, 7320, 9670, 14060]

# Calcular el modelo cuadrático
a, b, c = calcular_modelo_cuadratico(x, y)
print(f"Ecuación cuadrática: y = {a:.20f}x^2 + {b:.12f}x + {c:.6f}")
