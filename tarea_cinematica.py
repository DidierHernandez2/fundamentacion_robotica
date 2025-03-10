import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import sympy as sp

# Función para calcular la matriz de transformación homogénea A_i
def transformation_matrix(theta_i, d_i, a_i, alpha_i):
    alpha_i = np.radians(alpha_i)  # Convertir a radianes
    return np.array([
        [np.cos(theta_i), -np.sin(theta_i) * np.cos(alpha_i), np.sin(theta_i) * np.sin(alpha_i), a_i * np.cos(theta_i)],
        [np.sin(theta_i), np.cos(theta_i) * np.cos(alpha_i), -np.cos(theta_i) * np.sin(alpha_i), a_i * np.sin(theta_i)],
        [0, np.sin(alpha_i), np.cos(alpha_i), d_i],
        [0, 0, 0, 1]
    ])
def transformation_matrix_sym(theta_i, d_i, a_i, alpha_i):
    alpha_i = sp.rad(alpha_i)  # Convertir a radianes
    theta_i = sp.rad(theta_i)
    return sp.Matrix([
        [sp.cos(theta_i), -sp.sin(theta_i) * sp.cos(alpha_i), sp.sin(theta_i) * sp.sin(alpha_i), a_i * sp.cos(theta_i)],
        [sp.sin(theta_i), sp.cos(theta_i) * sp.cos(alpha_i), -sp.cos(theta_i) * sp.sin(alpha_i), a_i * sp.sin(theta_i)],
        [0, sp.sin(alpha_i), sp.cos(alpha_i), d_i],
        [0, 0, 0, 1]
    ])
# Definir los parámetros DH para el KUKA LBR iiwa 7 R800
a = [0, 280, 240, 0, 0, 0, 0]  # Longitudes de los eslabones (mm)
d = [340, 0, 0, 280, 0, 200, 0]  # Desplazamientos en z (mm)
alpha = [-90, 0, -90, 90, -90, 90, 0]  # Ángulos en grados

# Crear el modelo DH del robot
kuka = rtb.DHRobot([
    rtb.RevoluteDH(d=d[0], a=a[0], alpha=np.radians(alpha[0])),
    rtb.RevoluteDH(d=d[1], a=a[1], alpha=np.radians(alpha[1])),
    rtb.RevoluteDH(d=d[2], a=a[2], alpha=np.radians(alpha[2])),
    rtb.RevoluteDH(d=d[3], a=a[3], alpha=np.radians(alpha[3])),
    rtb.RevoluteDH(d=d[4], a=a[4], alpha=np.radians(alpha[4])),
    rtb.RevoluteDH(d=d[5], a=a[5], alpha=np.radians(alpha[5])),
    rtb.RevoluteDH(d=d[6], a=a[6], alpha=np.radians(alpha[6]))
], name="KUKA LBR iiwa 7 R800")

# Definir las configuraciones de las articulaciones
theta = np.radians([0, 0, 0, 0, 0, 0, 0])  # Todas en 0°

# Calcular las matrices de transformación individuales
T1 = transformation_matrix(theta[0], d[0], a[0], alpha[0])
T2 = transformation_matrix(theta[1], d[1], a[1], alpha[1])
T3 = transformation_matrix(theta[2], d[2], a[2], alpha[2])
T4 = transformation_matrix(theta[3], d[3], a[3], alpha[3])
T5 = transformation_matrix(theta[4], d[4], a[4], alpha[4])
T6 = transformation_matrix(theta[5], d[5], a[5], alpha[5])
# Multiplicar las matrices en orden
T12 = np.dot(T1, T2)
T123 = np.dot(T12, T3)
T1234 = np.dot(T123, T4)
T12345 = np.dot(T1234, T5)
T123456 = np.dot(T12345, T6)
# Imprimir matrices
print("Matriz de transformacion final para configuracion 1:")
print(T123456)
T1 = kuka.fkine(theta)
# Imprimir resultados
print("Cinemática Directa - Configuración 1:")
print(T1)
kuka.plot(theta, block=True)

#Configuracion 2
theta2 = np.radians([45, -45, 45, -45, 45, -45, 45])  # Alternadas 45° y -45°
T1 = transformation_matrix(theta2[0], d[0], a[0], alpha[0])
T2 = transformation_matrix(theta2[1], d[1], a[1], alpha[1])
T3 = transformation_matrix(theta2[2], d[2], a[2], alpha[2])
T4 = transformation_matrix(theta2[3], d[3], a[3], alpha[3])
T5 = transformation_matrix(theta2[4], d[4], a[4], alpha[4])
T6 = transformation_matrix(theta2[5], d[5], a[5], alpha[5])

T12 = np.dot(T1, T2)
T123 = np.dot(T12, T3)
T1234 = np.dot(T123, T4)
T12345 = np.dot(T1234, T5)
T123456 = np.dot(T12345, T6)

print("Matriz de transformacion final para configuracion 2:")
print(T123456)

T2 = kuka.fkine(theta2)

print("\nCinemática Directa - Configuración 2:")
print(T2)

kuka.plot(theta2, block=True)
#Configuracion 3
theta3 = np.radians([90, -90, 90, -90, 90, -90, 90])
T1 = transformation_matrix(theta3[0], d[0], a[0], alpha[0])
T2 = transformation_matrix(theta3[1], d[1], a[1], alpha[1])
T3 = transformation_matrix(theta3[2], d[2], a[2], alpha[2])
T4 = transformation_matrix(theta3[3], d[3], a[3], alpha[3])
T5 = transformation_matrix(theta3[4], d[4], a[4], alpha[4])
T6 = transformation_matrix(theta3[5], d[5], a[5], alpha[5])

T12 = np.dot(T1, T2)
T123 = np.dot(T12, T3)
T1234 = np.dot(T123, T4)
T12345 = np.dot(T1234, T5)
T123456 = np.dot(T12345, T6)

print("Matriz de transformacion final para configuracion 3:")
print(T123456)

T3 = kuka.fkine(theta3)
# Calcular la cinemática directa

print("\nCinemática Directa - Configuración 3:")
print(T3)

# Graficar las configuraciones del robot

kuka.plot(theta3, block=True)

theta_sym = sp.symbols('theta1:7')
T1 = transformation_matrix_sym(theta_sym[0], d[0], a[0], alpha[0])
T2 = transformation_matrix_sym(theta_sym[1], d[1], a[1], alpha[1])
T3 = transformation_matrix_sym(theta_sym[2], d[2], a[2], alpha[2])
T4 = transformation_matrix_sym(theta_sym[3], d[3], a[3], alpha[3])
T5 = transformation_matrix_sym(theta_sym[4], d[4], a[4], alpha[4])
T6 = transformation_matrix_sym(theta_sym[5], d[5], a[5], alpha[5])
# Multiplicar las matrices en orden de manera simbólica
T123456 = T1 * T2 * T3 * T4 * T5 * T6

# Imprimir la matriz de transformación simbólica final
sp.pprint(T123456)