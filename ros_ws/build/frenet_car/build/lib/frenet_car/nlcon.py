
import numpy as np


# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass

def nlcon(u, t0, t1, t2):

    # Local Variables: c, t2, t0, t1, u, ceq
    # Function calls: nlcon
    ceq = np.array([])
    c = t0+np.dot(u[0], t1)+np.dot(u[1], t2)
    return [c, ceq]

def main():
    print(nlcon([0,10],[10,2,3,4,5],[1,29,3,4,5],[1,2,3,4,54]))
  
if __name__== "__main__":
    main()
