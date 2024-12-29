#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs
import json
import regex as re
import os
import time

SPEED = 0.1 # Velocidad de la animación

# ******************************************************************************
# Declaración de funciones
def muestra_origenes(O,final=0):
  """
  Muestra los orígenes de coordenadas para cada articulación.
  Args:
    O (list): Lista de orígenes de coordenadas.
    final (list, optional): Posición final del efector final. Por defecto, es 0.
  """
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj, fig=None, ax=None):
  """
  Dibuja el robot y el objetivo en una gráfica interactiva.
  Parámetros:
    O (list): Lista de matrices de transformación homogénea que representan las posiciones de las articulaciones y enlaces.
    obj (tuple): Coordenadas del objetivo (x, y) que el robot debe alcanzar.
    fig (matplotlib.figure.Figure, opcional): Figura de matplotlib para el gráfico. Si es None, se crea una nueva figura.
    ax (matplotlib.axes.Axes, opcional): Ejes del gráfico. Si es None, se crean nuevos ejes.
  Retorna:
    fig (matplotlib.figure.Figure): Figura de matplotlib con el gráfico del robot.
    ax (matplotlib.axes.Axes): Ejes actualizados con el dibujo del robot.
  """
  graph_length = L[2]
  dist = np.linalg.norm(np.subtract(obj,O[0][0]))
  graph_length = max(graph_length, dist) + 0.5

  if fig is None or ax is None:
    fig, ax = plt.subplots()
    plt.ion()  # Activar modo interactivo
  # Limpiar la figura para redibujar
  ax.cla()  # Limpiar los ejes
  ax.set_xlim(-graph_length, graph_length)
  ax.set_ylim(-graph_length, graph_length)

  # Dibujar brazo
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    ax.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  
  # Dibujar objetivo
  ax.plot(obj[0], obj[1], '*')
  #plt.savefig('robot.png')
  plt.pause(5)
  return fig, ax

def matriz_T(d,th,a,al):
  """
  Calcula la matriz de transformación homogénea para una articulación.
  Parámetros:
    d (float): Desplazamiento a lo largo del eje z (longitud del enlace).
    th (float): Ángulo de rotación alrededor del eje z (en radianes).
    a (float): Longitud del enlace (distancia entre ejes x).
    al (float): Ángulo de torsión alrededor del eje x (en radianes).
  Retorna:
    list: Matriz de transformación homogénea 4x4 que describe la transformación de la articulación.
  """
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  """
  Calcula la cinemática directa.
  Parámetros:
    th (list): Lista de ángulos de las articulaciones de rotación.
    a (list): Lista de longitudes de los enlaces y desplazamientos de articulaciones prismáticas.
  Retorna:
    list: Lista de coordenadas (x, y) de las posiciones de las articulaciones y el efector final en el espacio.
  """
  T = np.identity(4)
  o = [[0,0]]

  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o
# ****************************************************************************** #
# Función que lee el archivo de configuración del brazo y devuelve la estructura #
def get_arm(file):
  """
  Lee el archivo de configuración JSON y extrae la información del brazo robótico.
  Parámetros:
    file (str): Ruta del archivo JSON que contiene la configuración del brazo.
  Retorna:
    dict: Diccionario con la estructura del brazo robótico, que incluye:
      - "joints" (list): Tipos de articulaciones (0: rotación, 1: desplazamiento).
      - "th" (list): Ángulos de las articulaciones de rotación.
      - "a" (list): Longitudes de los enlaces y desplazamientos prismáticos.
      - "limits" (list): Límites de las articulaciones.
  """
  with open(file, 'r') as f:
    data = json.load(f)

  joint_types = []
  th = []
  a = []          
  joint_limits = []
  last_joint_type = None

  def validate_and_append(limits, value, default_value, append_list):
    """
    Valida y agrega un valor a la lista correspondiente si está dentro de los límites permitidos.
    Parámetros:
      limits (list): Límites [mínimo, máximo] del valor.
      value (float): Valor actual de la articulación.
      default_value (list): Valor por defecto si los límites no están definidos.
      append_list (list): Lista a la que se añadirá el valor validado.
    Retorna:
      None: Actualiza las listas globales `joint_limits` y `append_list`.
    """
    if limits[0] == None: limits[0] = default_value[0]
    if limits[1] == None: limits[1] = default_value[1]

    if limits[0] <= value <= limits[1]:
      joint_limits.append(limits)
      append_list.append(value)
    else:
      sys.exit("Error(configuración del brazo): valor de " + str(key) + " fuera de los límites")

  for key, value in data.items():
    item_type = value.get("type")
    # Articulaciones (p0, p1, ...)
    if re.search("^p[0-9]*", key) and item_type == "joint":
      joint_type = value.get("move")
      joint_value = value.get("value", 0.0) # valor por defecto
      limits = value.get("limits", [None, None]) # limites por defecto
      if joint_type == "rotation":
        joint_types.append(0) # 0 para giro
        last_joint_type = 0
        # Paso de angulos a radianes (en el json estan en grados)
        if limits[0] is not None:
          limits[0] = radians(limits[0])
          limits[0] = normalize(limits[0])
        if limits[1] is not None:
          limits[1] = radians(limits[1])
          limits[1] = normalize(limits[1])
        joint_value = radians(joint_value)
        joint_value = normalize(joint_value)
        validate_and_append(limits, joint_value, [-pi, pi], th)
      elif joint_type == "displacement": 
        joint_types.append(1) # 1 para prismatico
        last_joint_type = 1
        validate_and_append(limits, joint_value, [0, 100], a)
        th.append(0) # angulo de articulacion es 0 (es prismatico)
      else:
        sys.exit("Error(configuración del brazo): tipo de articulación no reconocido")
    # Elementos rijidos (l0, l1, ...)
    elif re.search("^l[0-9]*", key) and item_type == "linker": 
      linker_value = value.get("value")
      if last_joint_type == 0:
          if linker_value >= 0:
            a.append(linker_value)
          else :
            sys.exit("Error(configuración del brazo): valor de "+ str(key) +" debe ser positivo")
    # Efector final (ef)
    elif key == "ef" and item_type == "final":
      continue # no se hace nada
    # Elementos no reconocidos
    else:
      sys.exit("Error(configuración del brazo): elemento "+ str(key) +" no reconocido.")
      exit()

  return {
    "joints": joint_types, 
    "th": th,              
    "a": a,                
    "limits": joint_limits 
  }

############################
## Funciones para el giro ##
############################
def theta_increment(P, EF, t): # u·v = |u||v|cos(theta)
  """
  Calcula el ángulo de incremento (theta) necesario para alinear una articulación hacia un punto objetivo.
  Parámetros:
    P (list): Coordenadas [x, y] de la articulación base.
    EF (list): Coordenadas [x, y] del efector final.
    t (list): Coordenadas [x, y] del punto objetivo.
  Retorna:
    float: Ángulo de incremento theta, positivo si es antihorario, negativo si es horario.
  """
  # vector u: articulacion a efector final
  u = [EF[0] - P[0], EF[1] - P[1]]
  mod_u = sqrt(pow(u[0], 2) + pow(u[1], 2))
  # vector v: articulacion a punto objetivo 
  v = [t[0] - P[0], t[1] - P[1]]
  mod_v = sqrt(pow(v[0], 2) + pow(v[1], 2))
  # Producto escalar u·v y cálculo del ángulo theta 
  dot_product = np.dot(u, v)
  coseno_theta = dot_product / (mod_u * mod_v)
  tolerance = 1e-10
  if abs(coseno_theta - 1) < tolerance: theta = 0
  else: theta = acos(coseno_theta)
  # Determinación del signo del ángulo usando el producto cruzado
  cross_product = np.cross(u, v)
  if cross_product > 0: 
    theta = abs(theta) # Antihorario
  else: 
    theta = -abs(theta) # Horario
  return theta

def normalize(theta):
  """
  Normaliza un ángulo para que esté en el rango [-pi, pi].
  Parámetros:
    theta (float): Ángulo a normalizar.
  Retorna:
    float: Ángulo normalizado en el rango [-pi, pi].
  """
  # si se sale del rango [-pi, pi] se normaliza
  if theta < -pi or theta > pi:
    if theta % pi == 0:
      if theta % (2 * pi) == 0:
        theta = 0
      else:
        sign = -1 if theta > 0 else 1
        theta = sign * pi
    else:
      theta = (theta + pi) % (2 * pi) - pi
  return theta

def arm_length(a, arm):
  """
  Calcula la longitud mínima y máxima del brazo considerando las longitudes de enlaces y límites de articulaciones prismáticas.
  Parámetros:
    a (list): Lista de longitudes de enlaces rígidos.
    arm (dict): Estructura del brazo con los datos de las articulaciones:
      - "joints" (list): Tipos de articulaciones (0: rotación, 1: desplazamiento).
      - "limits" (list): Límites [mínimo, máximo] de cada articulación.
  Retorna:
    list: Longitud mínima, máxima y actual del brazo.
  """
  # Calcula la longitud total del brazo
  arm_length = 0
  for value in a:
    arm_length += value
  # Agrega los limites de las prismaticas
  min = max = 0
  for i in range(len(arm["joints"])):
    if arm["joints"][i] == 1:
      min += arm["limits"][i][0]
      max += arm["limits"][i][1]
  return [arm_length - min, arm_length + max, arm_length]

###################################
## Funciones para las prismatica ##
###################################
def displacement_increment(EF, t, w):
  """
  Calcula el incremento necesario en la longitud de una articulación prismática para acercar el efector final al punto objetivo.
  Parámetros:
    EF (list): Coordenadas [x, y] del efector final.
    t (list): Coordenadas [x, y] del punto objetivo.
    w (float): Ángulo del enlace respecto al eje horizontal.
  Retorna:
    float: Incremento en la longitud de la articulación prismática.
  """
  # direccion unitaria del enlace
  u = [cos(w), sin(w)]
  # vector hacia objetivo
  v = [t[0] - EF[0], t[1] - EF[1]]
  # distancia al objetivo
  d = (v[0] * u[0] + v[1] * u[1]) # producto escalar
  return d

##########################
## Funciones auxiliares ##
##########################
# Comprobación de limites
def check_limit(value, limits):
  """
  Verifica si un valor de la articulación está dentro de los límites especificados y ajusta el valor si es necesario.
  Parámetros:
    value (float): Valor a comprobar (puede ser ángulo o desplazamiento).
    limits (list): Lista con los límites [mínimo, máximo].
  Retorna:
    float: Valor ajustado dentro de los límites.
  """
  if value < limits[0]: 
    return limits[0]
  if value > limits[1]: 
    return limits[1]
  return value

def is_reachable_objetive(objetive, arm_origin, arm_length):
  """
  Comprueba si el punto objetivo está dentro del alcance del brazo robótico.
  Parámetros:
    objetive (list): Coordenadas [x, y] del punto objetivo.
    arm_origin (list): Coordenadas [x, y] del origen del brazo robótico.
    arm_length (float): Longitud máxima del brazo robótico.
  Retorna:
    bool: True si el objetivo es alcanzable, detiene la ejecución en caso contrario.
  """
  # distancia euclidea del origen al objetivo
  distance = np.linalg.norm(np.subtract(objetive, arm_origin))
  if distance > arm_length:
    print("Distancia al objetivo: " + str(round(distance, 3)))
    print("El punto objetivo no es alcanzable")
    #(O, objetive)
    sys.exit()
  return True

# ********************************************************************* #
# Cálculo de la cinemática inversa de forma iterativa por el método CCD #
# ********************************************************************* #
plt.ion() # Activar modo interactivo para visualización dinámica.

# Validación de entrada: 
#  - Se espera un archivo JSON con la configuración del brazo
#    y coordenadas objetivo.
if len(sys.argv) != 4:
  sys.exit("Uso: python " + sys.argv[0] + " configuracion.json x y")

# Punto objetivo (x, y)
objetive=[float(i) for i in sys.argv[2:]]

# Cargar configuracion del brazo
arm = get_arm(sys.argv[1])
joints = arm["joints"] # Angulos de las articulaciones
th = arm["th"]         # Longitudes de enlaces y desplazamientos
a = arm["a"]           # Tipos de articulaciones (0: rotación, 1: desplazamiento)
limits = arm["limits"] # Límites de las articulaciones
EPSILON = .01          # Umbral para determinar convergencia.
# Calculo de la cinemática directa
O=cin_dir(th,a)

# Muestra la posicion inicial
#print ("- Posicion inicial:")
#muestra_origenes(O)

# Inicializar variables para el cálculo iterativo.
dist = float("inf") # Distancia al objetivo
prev = 0.
iteracion = 1

# Comprobar si el objetivo es alcanzable
L = arm_length(a, arm)
is_reachable = is_reachable_objetive(objetive, O[0], L[1])
if not is_reachable_objetive(objetive, O[0], L[1]):
  sys.exit("El punto objetivo no es alcanzable, por favor, cambie las coordenadas objetivo.")

fig, ax = None, None # Para la visualización del robot
start = time.time() # Iniciar temporizador
# Bucle iterativo para resolver la cinemática inversa.
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  O=[cin_dir(th,a)]
  # Iterar sobre las articulaciones
  for i in range(len(joints)):
    currentO = O[-1] # configuracion actual
    i = i + 1

    # Cálculo de la cinemática inversa según el tipo de articulación:
    ## Articulaciones de giro
    if joints[-i] == 0:
      theta = theta_increment(currentO[-(i+1)], currentO[-1], objetive)
      th[-i] += theta
      th[-i] = normalize(th[-i])
      th[-i] = check_limit(th[-i], limits[-i])
    ## Articulaciones prismáticas
    else:
      w = sum(th[:-i]) # angulos de giro anteriores
      d = displacement_increment(currentO[-1], objetive, w)
      a[-i] += d
      a[-i] = check_limit(a[-i], limits[-i])
    # Actualizar la configuración del brazo
    O.append(cin_dir(th,a))
    
  # Mostrar la configuración actual y dibujar el robot
  dist = np.linalg.norm(np.subtract(objetive,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  #muestra_origenes(O[-1])
  fig, ax = muestra_robot(O,objetive, fig, ax)
  #print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

end = time.time() # Parar temporizador
print("\nTiempo de ejecución: " + str(round(end - start, 3)) + " segundos.")

# Evaluar el resultado final tras salir del bucle.
if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
# Mostrar estadísticas finales.
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
plt.show()
input("Pulse Enter para finalizar...")