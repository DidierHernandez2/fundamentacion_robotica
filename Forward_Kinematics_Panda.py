import roboticstoolbox as rtb
import numpy as np

panda=rtb.models.ETS.Panda()
#qr Ise the angular position by default
print("Default angular position qr")
print(panda.qr)
print("Homogeneus Transformation Matrix")
print(panda.fkine(panda.qr))

panda_dh=rtb.models.DH.Panda()
print(panda_dh)

panda.plot(panda.qr, block=True)

new_pos=np.array([0, 0, 0, 0, 0, 0, 0])
panda.plot(new_pos, block=True)
print(panda.fkine(new_pos))
