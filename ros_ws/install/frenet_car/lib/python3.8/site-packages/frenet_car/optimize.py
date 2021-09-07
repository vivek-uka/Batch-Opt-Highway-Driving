
import numpy as np
from scipy.optimize import minimize
import NonLinearConstraint, term1, term2, nlcon
from cvxopt import matrix, solvers

def optimize(head, vr, wr, xr, yr, xob, yob, xobdot, yobdot, R, dt, lb, ub, objective):

    # Local Variables: robovx, vr, xr, lb, yob, relpx, relpy, xob, objective, opt, robovy, head, nonlin, relvx, relvy, R, wr, dt, yr, yobdot, t2, t0, t1, u, xobdot, ub
    # Function calls: cos, NonLinearConstraint, term2, term1, sin, fmincon, optimize, nlcon
    relpx = xr-xob
    relpy = yr-yob
    relvx = np.dot(vr, np.cos(head))-xobdot
    relvy = np.dot(vr, np.sin(head))-yobdot
    robovx = np.dot(vr, np.cos(head))
    robovy = np.dot(vr, np.sin(head))
    t0 = NonLinearConstraint.NonLinearConstraint(np.array(np.hstack((0., 0.))), head, vr, wr, relpx, relpy, relvx, relvy, robovx, robovy, R, dt)
    t1 = term1.term1(np.array(np.hstack((0., 0.))), head, vr, wr, relpx, relpy, relvx, relvy, robovx, robovy, R, dt)
    t2 = term2.term2(np.array(np.hstack((0., 0.))), head, vr, wr, relpx, relpy, relvx, relvy, robovx, robovy, R, dt)
    nonlin = lambda u: nlcon.nlcon(u, t0, t1, t2)
    bnds = ((lb[0], ub[0]), (lb[1], ub[1]))
    cons = ({'type': 'ineq', 'fun': lambda u: -t0[0] - t1[0]*u[0] - t2[0]*u[1]})
    x0=((ub+lb)/2)
    opt = minimize(objective,x0, method='SLSQP', bounds=bnds, constraints=cons)
    return opt.x
