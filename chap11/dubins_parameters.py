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
    ell = np.linalg.norm(ps-pe)
    if ell < 2 * R:
        print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
    else:
        # compute start and end circles
        crs = R * rotz(np.pi/2) @ np.array([[np.cos(chis)],[np.sin(chis)],[0.0]]) + ps
        cls = R * rotz(-np.pi/2) @ np.array([[np.cos(chis)],[np.sin(chis)],[0.0]]) + ps
        cre = R * rotz(np.pi/2) @ np.array([[np.cos(chie)],[np.sin(chie)],[0.0]]) + pe
        cle = R * rotz(-np.pi/2) @ np.array([[np.cos(chie)],[np.sin(chie)],[0.0]]) + pe

        # compute L1
        # tempvar1 = (cre.item(0)-crs.item(0))
        # tempvar2 = np.linalg.norm(cre-crs)
        vartheta = np.arctan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0)) #np.arccos(tempvar1/tempvar2)
        L1 = np.linalg.norm(crs - cre) + R * mod(2.0*np.pi + mod(vartheta - np.pi/2.0) - mod(chis-np.pi/2.0))                  \
                                + R * mod(2.0*np.pi + mod(chie - np.pi/2.0) - mod(vartheta-np.pi/2.0))
        # compute L2

        ell = np.linalg.norm(cle-crs) 
        vartheta = np.arctan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))#np.arccos((cle.item(0)-crs.item(0))/np.linalg.norm(cle-crs))
        vartheta2 = vartheta - np.pi/2.0 + np.arcsin(2.0*R / ell)
        if not np.isreal(vartheta2):
            vartheta2 = vartheta
            L2 = np.sqrt(ell**2 - 4.0* R**2) + R * mod(2.0* np.pi + mod(vartheta2) - mod(chis - np.pi/2.0))             \
                                             + R * mod(2.0* np.pi + mod(vartheta2 +np.pi) - mod(chie + np.pi/2.0)) 
        else:
            L2 = np.sqrt(ell**2 - 4.0* R**2) + R * mod(2.0* np.pi + mod(vartheta2) - mod(chis - np.pi/2.0))             \
                                             + R * mod(2.0* np.pi + mod(vartheta2 +np.pi) - mod(chie + np.pi/2.0))  

        # compute L3
        ell = np.linalg.norm(cre-cls) 
        vartheta = np.arctan2(cre.item(1)-cls.item(1), cre.item(0)-cls.item(0))#np.arccos((cre.item(0)-cls.item(0))/np.linalg.norm(cre-cls))
        vartheta2 = np.arccos(2.0*R / ell)
        if not np.isreal(vartheta2):
            vartheta2 = 0.0
            L3 = np.sqrt(ell**2 - 4.0*R**2) + R * mod(2.0*np.pi + mod(chis+np.pi/2.0) - mod(vartheta+vartheta2))         \
                                            + R * mod(2*np.pi + mod(chie - np.pi/2.0) - mod(vartheta + vartheta2 - np.pi))
        else:
            L3 = np.sqrt(ell**2 - 4.0*R**2) + R * mod(2.0*np.pi + mod(chis+np.pi/2.0) - mod(vartheta+vartheta2))         \
                                            + R * mod(2*np.pi + mod(chie - np.pi/2.0) - mod(vartheta + vartheta2 - np.pi))
        # compute L4
        vartheta = np.arctan2(cle.item(1)-cls.item(1), cle.item(0)-cls.item(0))#np.arccos((cle.item(0)-cls.item(0))/np.linalg.norm(cle-cls))
        L4 = np.linalg.norm(cls-cle) + R * mod(2.0*np.pi + mod(chis+np.pi/2.0) - mod(vartheta + np.pi/2.0))                      \
                              + R * mod(2.0*np.pi + mod(vartheta + np.pi/2.0) - mod(chie+np.pi/2.0)) 
        e1 = np.array([[1.0],[0.0],[0.0]])
        # L is the minimum distance
        L = np.min([L1, L2, L3, L4])
        idx = np.argmin([L1, L2, L3, L4])
        if idx == 0:
            cs = crs
            lams = 1 
            ce = cre
            lame = 1
            q1 = (ce-cs)/np.linalg.norm(ce-cs)
            w1 = cs + R * rotz(-np.pi/2.0)@q1
            w2 = ce + R * rotz(-np.pi/2.0)@q1
        elif idx == 1:
            cs = crs
            lams = 1
            ce = cle
            lame = -1
            ell = np.linalg.norm(ce-cs)
            vartheta = np.arctan2(ce.item(1)-cs.item(1), ce.item(0)-cs.item(0))#np.arccos((ce.item(0)-cs.item(0))/ell)
            vartheta2 = vartheta - np.pi/2.0 + np.arcsin(2.0*R / ell)
            q1 = rotz(vartheta2 +np.pi/2.0)@e1
            w1 = cs + R * rotz(vartheta2)@e1
            w2 =  ce + R * rotz(vartheta2 + np.pi)@e1
        elif idx == 2:
            cs = cls
            lams = -1
            ce = cre
            lame = 1
            ell = np.linalg.norm(ce-cs)
            vartheta = np.arctan2(ce.item(1)-cs.item(1), ce.item(0)-cs.item(0))#np.arccos((ce.item(0)-cs.item(0))/ell)
            vartheta2 = np.arccos(2*R/ell)
            q1 = rotz(vartheta + vartheta2 - np.pi/2.0) @ e1
            w1 = cs + R * rotz(vartheta + vartheta2) @ e1
            w2 = ce + R * rotz(vartheta + vartheta2 - np.pi) @ e1
        elif idx == 3:
            cs = cls
            lams = -1
            ce = cle
            lame = -1
            q1 = (ce-cs) / np.linalg.norm(ce-cs)
            w1 = cs + R * rotz(np.pi/2.0)@q1
            w2 = ce + R * rotz(np.pi/2.0)@q1
        w3 = pe
        q3 = rotz(chie) @ e1

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


