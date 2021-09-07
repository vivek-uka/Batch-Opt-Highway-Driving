
import numpy as np


# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass

def NonLinearConstraint(u, head, vr, wr, relpx, relpy, relvx, relvy, robovx, robovy, R, dt):

    # Local Variables: robovx, robovy, head, relvx, f, i, relvy, relpx, relpy, vr, R, u, wr, lindelta, dt, angdelta, t
    # Function calls: length, NonLinearConstraint, cos, sin
    lindelta = u[0]
    angdelta = u[1]
    f = []
    for i in np.arange((len(relpx))):
        t = (relpx[i]*(relvx[i]+-1.*robovx+(lindelta+vr)*np.cos((head+dt*(angdelta+wr))))+relpy[i]*(relvy[i]+-1.*robovy+(lindelta+vr)*np.sin((head+dt*(angdelta+wr)))))**2.+(R**2.+-1.*relpx[i]**2.+-1.*relpy[i]**2.)*((relvx[i]+-1.*robovx+(lindelta+vr)*np.cos((head+dt*(angdelta+wr))))**2.+(relvy[i]+-1.*robovy+(lindelta+vr)*np.sin((head+dt*(angdelta+wr))))**2.)
        f.append(t)
        
    return f

def main():
    print(NonLinearConstraint([0,0], 0.78, 1, 0, [6], [6], [-1], [-1], 1, 1, 1, 0.1))
  
if __name__== "__main__":
    main()
