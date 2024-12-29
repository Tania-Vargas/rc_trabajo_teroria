# Repositorio de Algoritmos CCD y FABRIK

Este repositorio contiene implementaciones de dos algoritmos de cinemática inversa utilizados en el ámbito de la robótica: **CCD** (Cyclic Coordinate Descent) y **FABRIK** (Forward and Backward Reaching Inverse Kinematics). También se incluyen ejemplos de configuraciones de brazo robótico en formato JSON para su uso con estos algoritmos.

## Contenido del Repositorio

### Archivos Principales

- **`ccdp3.py`**: Implementación del algoritmo CCD utilizada para la segunda práctica entregada en clase.
- **`fabrik.py`**: Implementación del algoritmo FABRIK desarrollada para un trabajo teórico.

### Configuraciones de Brazo

- **Ejemplo 1**: `brazo2.json`
- **Ejemplo 2**: `brazo3.json`

Ambos archivos contienen configuraciones de brazo robótico, incluyendo los tipos de articulaciones y sus límites.

## Ejecución

La forma de ejecutar ambos algoritmos es la siguiente:

```bash
python <codigo ccd o fabrik> <configuracion de brazo (archivo JSON)> <coordenada x> <coordenada y>
```

### Ejemplo de Ejecución

Para el algoritmo CCD con la configuración del brazo en `brazo1.json` y una posición objetivo en (5, 7):

```bash
python ccdp3.py brazo1.json 5 7
```

## Diferencias entre las Implementaciones

### CCD (`ccdp3.py`)

- **Tipos de Articulaciones**: Soporta tanto articulaciones de giro (rotacionales) como de desplazamiento (prismáticas).
- **Verificación de Límites**: Los límites de las articulaciones se comprueban en cada movimiento.
- **Normalización de Ángulos**: Se realiza para garantizar que los valores estén dentro de los rangos permitidos.

### FABRIK (`fabrik.py`)

- **Tipos de Articulaciones**: Solo soporta articulaciones de giro (rotacionales).
- **Verificación de Límites**: Los límites de las articulaciones se comprueban únicamente durante la lectura inicial de la configuración del brazo.
- **Normalización de Ángulos**: También implementada.

## Notas Adicionales

Ambos algoritmos leen la configuración del brazo robótico desde un archivo JSON. Es importante asegurarse de que las configuraciones sean válidas y que los límites estén correctamente definidos antes de ejecutar los programas.
