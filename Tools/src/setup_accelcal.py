from scipy.optimize import minimize
import numpy as np
from math import *

GRAVITY_MSS = 9.80655

def column(array):
    return np.matrix(np.squeeze(np.asarray(array))).T

def correct_sample(p, s):
    p = np.squeeze(np.asarray(p))
    s = column(s)
    bias = np.matrix([p[0], p[1], p[2]]).T

    mat = np.matrix([
        [ p[3] ,  0.  ,  0.  ],
        [  0.  , p[4] ,  0.  ],
        [  0.  ,  0.  , p[5] ]
        ])

    return mat*(s-bias)

def calc_residual(p, s):
    return GRAVITY_MSS - np.linalg.norm(correct_sample(p, s))

def calc_jacobian_6dof(p, s):
    p = np.squeeze(np.asarray(p))
    ret = np.zeros(6)

    for sample in s:
        sample = column(sample)
        ret += np.array([
            (2*p[3]**2*(sample.item(0)-p[0])*(GRAVITY_MSS-sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)))/sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2),
            (2*p[4]**2*(sample.item(1)-p[1])*(GRAVITY_MSS-sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)))/sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2),
            (2*p[5]**2*(sample.item(2)-p[2])*(GRAVITY_MSS-sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)))/sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2),
            -(2*p[3]*(sample.item(0)-p[0])**2*(GRAVITY_MSS-sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)))/sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2),
            -(2*p[4]*(sample.item(1)-p[1])**2*(GRAVITY_MSS-sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)))/sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2),
            -(2*p[5]*(sample.item(2)-p[2])**2*(GRAVITY_MSS-sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)))/sqrt(p[3]**2*(sample.item(0)-p[0])**2+p[4]**2*(sample.item(1)-p[1])**2+p[5]**2*(sample.item(2)-p[2])**2)
            ])
    ret /= len(s)
    return np.squeeze(np.asarray(ret))

def calc_mean_squared_residuals_6dof(p, s):
    p = np.squeeze(np.asarray(p))
    residuals = []
    for sample in s:
        residuals.append(calc_residual(p,sample))
    return np.average(np.array(residuals)**2)

def calibrate_accel_6dof(samples):
    initial_params = [0.0,0.0,0.0,1.0,1.0,1.0]

    bounds = (
        (-0.5*GRAVITY_MSS,0.5*GRAVITY_MSS),
        (-0.5*GRAVITY_MSS,0.5*GRAVITY_MSS),
        (-0.5*GRAVITY_MSS,0.5*GRAVITY_MSS),
        (0.9,1.1),
        (0.9,1.1),
        (0.9,1.1)
        )

    return minimize(fun=calc_mean_squared_residuals_6dof, jac=calc_jacobian_6dof, x0=(initial_params,), args=(samples,), bounds=bounds).x

def calc_level_euler_rpy(p, s):
    corrected_sample = column(correct_sample(p, s))
    return column([
        atan2(-corrected_sample.item(1), -corrected_sample.item(2)),
        atan2(corrected_sample.item(0), np.linalg.norm(corrected_sample[1:,])),
        0
        ])
