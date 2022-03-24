# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB
#         4/2/2020 - RWB

import numpy as np
import sys
sys.path.append('..')


class DubinsParameters:
    def __init__(self, ps=9999*np.ones((3,1)), chis=9999,
                 pe=9999*np.ones((3,1)), chie=9999, R=9999):
        if R == 9999:
            L = R
            cs = ps
            lams = R
            ce = ps
            lame = R
            w1 = ps
            q1 = ps
            w2 = ps
            w3 = ps
            q3 = ps
        else:
            L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
                = compute_parameters(ps, chis, pe, chie, R)
        self.p_s = ps
        self.chi_s = chis
        self.p_e = pe
        self.chi_e = chie
        self.radius = R
        self.length = L
        self.center_s = cs
        self.dir_s = lams
        self.center_e = ce
        self.dir_e = lame
        self.r1 = w1
        self.n1 = q1
        self.r2 = w2
        self.r3 = w3
        self.n3 = q3

    def update(self, ps, chis, pe, chie, R):
         L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
            = compute_parameters(ps, chis, pe, chie, R)
         self.p_s = ps
         self.chi_s = chis
         self.p_e = pe
         self.chi_e = chie
         self.radius = R
         self.length = L
         self.center_s = cs
         self.dir_s = lams
         self.center_e = ce
         self.dir_e = lame
         self.r1 = w1
         self.n1 = q1
         self.r2 = w2
         self.r3 = w3
         self.n3 = q3


def compute_parameters(ps, chis, pe, chie, R):
    ell = np.norm(ps-pe)
    if ell < 2 * R:
        print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
    else:
        # compute start and end circles
        crs = ps + R * rotz(np.pi/2) @ np.array([[np.cos(chis)],[np.sin(chis)],[0.0]])
        cls = ps + R * rotz(-np.pi/2) @ np.array([[np.cos(chis)],[np.sin(chis)],[0.0]])
        cre = pe + R * rotz(np.pi/2) @ np.array([[np.cos(chie)],[np.sin(chie)],[0.0]])
        cle = pe + R * rotz(-np.pi/2) @ np.array([[np.cos(chie)],[np.sin(chie)],[0.0]])

        # compute L1
        vartheta = np.arccos((cre.item(0)-crs.item(0))/np.norm(cre-crs))
        L1 = np.norm(crs - cre) + R * mod(2.0*np.pi + mod(vartheta - np.pi/2.0) - mod(chis-np.pi/2.0))                  \
                                + R * mod(2.0*np.pi + mod(chie - np.pi/2.0) - mod(vartheta-np.pi/2.0))
        # compute L2

        ell = np.norm(cle-crs) 
        vartheta = np.arccos((cle.item(0)-crs.item(0))/np.norm(cle-crs))
        vartheta2 = vartheta - np.pi/2.0 + np.arcsin(2.0*R / ell)
        if not np.isreal(vartheta2):
            vartheta2 = vartheta
            L2 = np.sqrt(ell**2 - 4.0* R**2) + R * mod(2.0* np.pi + mod(vartheta2) - mod(chis - np.pi/2.0))             \
                                             + R * mod(2.0* np.pi + mod(vartheta2 +np.pi) - mod(chie + np.pi/2.0)) 
        else:
            L2 = np.sqrt(ell**2 - 4.0* R**2) + R * mod(2.0* np.pi + mod(vartheta2) - mod(chis - np.pi/2.0))             \
                                             + R * mod(2.0* np.pi + mod(vartheta2 +np.pi) - mod(chie + np.pi/2.0))  

        # compute L3
        ell = np.norm(cre-cls) 
        vartheta = np.arccos((cre.item(0)-cls.item(0))/np.norm(cre-cls))
        vartheta2 = np.arccos(2.0*R / ell)
        if not np.isreal(vartheta2):
            vartheta2 = 0.0
            L3 = np.sqrt(ell**2 - 4.0*R**2) + R * mod(2.0*np.pi + mod(chis+np.pi/2.0) - mod(vartheta+vartheta2))         \
                                            + R * mod(2*np.pi + mod(chie - np.pi/2.0) - mod(vartheta + vartheta2 - np.pi))
        else:
            L3 = np.sqrt(ell**2 - 4.0*R**2) + R * mod(2.0*np.pi + mod(chis+np.pi/2.0) - mod(vartheta+vartheta2))         \
                                            + R * mod(2*np.pi + mod(chie - np.pi/2.0) - mod(vartheta + vartheta2 - np.pi))
        # compute L4
        theta = np.arccos((cle.item(0)-cls.item(0))/np.norm(cle-cls))
        L4 = np.norm(cls-cle) + R * mod(2.0*np.pi + mod(chis+np.pi/2.0) - mod(vartheta + np.pi/2.0))                      \
                              + R * mod(2.0*np.pi + mod(vartheta + np.pi/2.0) - mod(chie+np.pi/2.0)) 
        # L is the minimum distance
        L = np.min([L1, L2, L3, L4])
        idx = np.argmin([L1, L2, L3, L4])
        if idx == 0:
            cs = 
            lams = 
            ce = 
            lame = 
            q1 = 
            w1 = 
            w2 = 
        elif idx == 1:
            cs = 
            lams = 
            ce = 
            lame = 
            q1 = 
            w1 = 
            w2 =  
        elif idx == 2:
            cs = 
            lams = 
            ce = 
            lame = 
            q1 = 
            w1 = 
            w2 = 
        elif idx == 3:
            cs = 
            lams = 
            ce = 
            lame = 
            q1 = 
            w1 = 
            w2 =  
        w3 = 
        q3 = 

        return L, cs, lams, ce, lame, w1, q1, w2, w3, q3


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


