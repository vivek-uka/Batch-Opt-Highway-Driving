
import numpy as np


# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass

def term2(u, head, vr, wr, relpx, relpy, relvx, relvy, robovx, robovy, R, dt):

    # Local Variables: robovx, robovy, head, relvx, f, i, relvy, relpx, relpy, vr, R, u, wr, lindelta, dt, angdelta, t
    # Function calls: cos, length, term2, sin
    lindelta = u[0]
    angdelta = u[1]
    f = []
    for i in np.arange((len(relpx))):
        t = np.dot(2., dt)*(lindelta+vr)*((R**2.+-1.*relpx[i]**2.+-1.*relpy[i]**2.)*((relvy[i]+-1.*robovy)*np.cos((head+dt*(angdelta+wr)))+(-1.*relvx[i]+robovx)*np.sin((head+dt*(angdelta+wr))))+(relpy[i]*np.cos((head+dt*(angdelta+wr)))+-1.*relpx[i]*np.sin((head+dt*(angdelta+wr))))*(relpx[i]*(relvx[i]+-1.*robovx+(lindelta+vr)*np.cos((head+dt*(angdelta+wr))))+relpy[i]*(relvy[i]+-1.*robovy+(lindelta+vr)*np.sin((head+dt*(angdelta+wr))))))
        f.append(t)
        
    return f

def main():
    print(term2([0,0], 0.78, 1, 0, [6], [6], [-1], [-1], 1, 1, 1, 0.1))
  
if __name__== "__main__":
    main()
