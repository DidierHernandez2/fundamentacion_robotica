import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

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
config_1 = np.radians([0, 0, 0, 0, 0, 0, 0])  # Todas en 0°
config_2 = np.radians([45, -45, 45, -45, 45, -45, 45])  # Alternadas 45° y -45°
config_3 = np.radians([30, -60, 45, -30, 60, -45, 15])  # Configuración sugerida

# Calcular la cinemática directa
T1 = kuka.fkine(config_1)
T2 = kuka.fkine(config_2)
T3 = kuka.fkine(config_3)

# Imprimir resultados
print("Cinemática Directa - Configuración 1:")
print(T1)

print("\nCinemática Directa - Configuración 2:")
print(T2)

print("\nCinemática Directa - Configuración 3:")
print(T3)

# Graficar las configuraciones del robot
kuka.plot(config_1, block=True)
kuka.plot(config_2, block=True)
kuka.plot(config_3, block=True)
