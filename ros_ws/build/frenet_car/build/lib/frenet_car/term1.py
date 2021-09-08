
import numpy as np

# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass

def term1(u, head, vr, wr, relpx, relpy, relvx, relvy, robovx, robovy, R, dt):

    # Local Variables: robovx, robovy, head, relvx, f, i, relvy, relpx, relpy, vr, R, u, wr, lindelta, dt, angdelta, t
    # Function calls: length, cos, term1, sin
    lindelta = u[0]
    angdelta = u[1]
    f = []
    for i in np.arange((len(relpx))):
        t = np.dot(2., R**2.+-1.*relpx[i]**2.+-1.*relpy[i]**2.)*(lindelta+vr+(relvx[i]+-1.*robovx)*np.cos((head+dt*(angdelta+wr)))+(relvy[i]+-1.*robovy)*np.sin((head+dt*(angdelta+wr))))+np.dot(2., relpx[i]*np.cos((head+dt*(angdelta+wr)))+relpy[i]*np.sin((head+dt*(angdelta+wr))))*(relpx[i]*(relvx[i]+-1.*robovx+(lindelta+vr)*np.cos((head+dt*(angdelta+wr))))+relpy[i]*(relvy[i]+-1.*robovy+(lindelta+vr)*np.sin((head+dt*(angdelta+wr)))))
        f.append(t)

    return f

def main():
    print(term1([0,0], 0.78, 1, 0, [6], [6], [-1], [-1], 1, 1, 1, 0.1))
  
if __name__== "__main__":
    main()
