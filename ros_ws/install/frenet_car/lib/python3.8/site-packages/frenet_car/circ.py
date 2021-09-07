
import numpy as np


# if available import pylab (from matlibplot)
try:
    import matplotlib.pylab as plt
except ImportError:
    pass

def circ(x, y, r,color):

    th = np.arange(0., (2.*np.pi), np.pi/50.)
    xunit = np.dot(r, np.cos(th))+x
    yunit = np.dot(r, np.sin(th))+y
    plt.plot(xunit, yunit,'black')
    plt.fill(xunit, yunit,color)




