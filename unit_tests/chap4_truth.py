"""
Expected values for mav_dynamics in chap4
"""
import numpy as np

# Dynamics
thrust = 24.72401373895999
torque = -1.2648175301140714
forcesAndMoments_e = np.array( [[ 3.39564899],
                                [ 0.        ],
                                [63.33373751],
                                [-0.29809076],
                                [ 8.75643373],
                                [ 0.        ]])
forcesAndMoments_a = np.array( [[ 2.80712493],
                                [ 1.63478906],
                                [57.66646876],
                                [10.43161838],
                                [ 0.5589213 ],
                                [-0.6942753 ]])
forcesAndMoments_r = np.array( [[ 2.80712493],
                                [ 0.41414656],
                                [57.66646876],
                                [-0.28294294],
                                [ 0.5589213 ],
                                [-0.43549996]])

# Wind
Va_n = 20.0
alpha_n = 0.0
beta_n = 0.0

Va_s = 30.0
alpha_s = 0.0
beta_s = 0.0

Va_e = 25.49509757
alpha_e = 0.0
beta_e = -0.19739556

Va_w = 25.49509757
alpha_w = 0.0
beta_w = 0.19739556

Va_u = 25.49509757
alpha_u = -0.19739556
beta_u = 0.0

Va_d = 25.49509757
alpha_d = 0.19739556
beta_d = 0.0
