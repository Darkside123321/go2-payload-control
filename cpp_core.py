from pathlib import Path
import ctypes
import numpy as np
LIB=ctypes.CDLL(str(Path(__file__).parent/'build/libcontrol_core.so'))
Arr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
Iarr=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
LIB.centroidal_B.argtypes=[ctypes.c_double,Arr,Arr,ctypes.c_double,Arr]
LIB.gait.argtypes=[ctypes.c_double,ctypes.c_double,ctypes.c_double,Iarr,Arr]
LIB.swing_ref.argtypes=[Arr,Arr,ctypes.c_double,ctypes.c_double,ctypes.c_double,Arr,Arr,Arr]
PERIOD=.5
DUTY=.65
def model_B(mass,invI,arms,dt):
    B=np.zeros((12,12))
    code=LIB.centroidal_B(mass,np.ascontiguousarray(invI),np.ascontiguousarray(arms),dt,B)
    if code:raise ValueError('invalid model mass or dt')
    return B
def schedule(time):
    contact=np.zeros(4,dtype=np.int32);phase=np.zeros(4)
    LIB.gait(float(time),PERIOD,DUTY,contact,phase)
    return contact.astype(bool),phase
def swing(p0,p1,phase,height=.045):
    p=np.zeros(3);v=p.copy();a=p.copy()
    LIB.swing_ref(np.ascontiguousarray(p0),np.ascontiguousarray(p1),float(phase),PERIOD*(1-DUTY),height,p,v,a)
    return p,v,a
