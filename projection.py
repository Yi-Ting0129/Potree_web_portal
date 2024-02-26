import numpy as np
from scipy import ndimage as ndi
from skimage import feature
from skimage.color import rgb2gray
from skimage.exposure import equalize_adapthist
from skimage.draw import polygon2mask, line
from skimage.restoration import denoise_bilateral
from skimage.measure import LineModelND, ransac, label, regionprops_table
from skimage.morphology import dilation, disk
from skimage.util import map_array
import pandas as pd
import time


class metrics(object):

    # basic function

    def rotation(self, b_w, b_p, b_k):

        if np.isscalar(b_w):
            R = np.zeros((1, 3, 3))
        else:
            R = np.zeros((b_w.size, 3, 3))

        omega = np.radians(b_w)
        phi = np.radians(b_p)
        ka = np.radians(b_k)

        R[:, 0, 0] = np.cos(phi) * np.cos(ka)
        R[:, 0, 1] = -np.cos(phi) * np.sin(ka)
        R[:, 0, 2] = np.sin(phi)

        R[:, 1, 0] = np.cos(omega) * np.sin(ka) + np.sin(omega) * np.sin(phi) * np.cos(ka)
        R[:, 1, 1] = np.cos(omega) * np.cos(ka) - np.sin(omega) * np.sin(phi) * np.sin(ka)
        R[:, 1, 2] = -np.sin(omega) * np.cos(phi)

        R[:, 2, 0] = np.sin(omega) * np.sin(ka) - np.cos(omega) * np.sin(phi) * np.cos(ka)
        R[:, 2, 1] = np.sin(omega) * np.cos(ka) + np.cos(omega) * np.sin(phi) * np.sin(ka)
        R[:, 2, 2] = np.cos(omega) * np.cos(phi)

        return R

    def iop_metrics(self, l_x, l_y, l_z, b_w, b_p, b_k):
        R_c_b = self.rotation(float(b_w), float(b_p), float(b_k))
        r_c_b = np.array([[[float(l_x)], [float(l_y)], [float(l_z)]]])

        return R_c_b, r_c_b

    def eop_metrics(self, eop, sensor):

        w = eop[:, 3]
        p = eop[:, 4]
        k = eop[:, 5]

        if sensor == 0:
            R_b_m = self.rotation(w, p, k)
        else:
            zeros = np.zeros(w.size)
            R_b_m = self.rotation(180, 0, 0) @ self.rotation(0, 0, -90) @ self.rotation(zeros, zeros, k) @ self.rotation(zeros, p, zeros) @ self.rotation(w, zeros, zeros)

        r_b_m = np.zeros((eop[:, 1].size, 3, 1))
        r_b_m[:, 0, :] = np.array(eop[:, [0]])
        r_b_m[:, 1, :] = np.array(eop[:, [1]])
        r_b_m[:, 2, :] = np.array(eop[:, [2]])

        return R_b_m, r_b_m

    def distortion_free(self, x, y, k1, k2, k3, p1, p2, r0):

        # This function has the measured image coordinates X, Y reduced to the principal
        # point as an input and outputs them distortion free.

        delx = x
        dely = y

        radius2 = np.power(delx, 2) + np.power(dely, 2)
        radius4 = np.power(radius2, 2)
        radius6 = np.power(radius2, 3)

        r02 = np.power(r0, 2)
        r04 = np.power(r02, 2)
        r06 = np.power(r02, 3)

        dist_x = k1 * np.multiply((radius2 - r02), delx) \
                 + k2 * np.multiply((radius4 - r04), delx) \
                 + k3 * np.multiply((radius6 - r06), delx) \
                 + p1 * (radius2 + 2 * np.power(delx, 2)) \
                 + p2 * 2.0 * np.multiply(delx, dely)
        dist_y = k1 * np.multiply((radius2 - r02), dely) \
                 + k2 * np.multiply((radius4 - r04), dely) \
                 + k3 * np.multiply((radius6 - r06), dely) \
                 + p1 * 2.0 * np.multiply(delx, dely) \
                 + p2 * (radius2 + 2.0 * np.power(dely, 2))

        x_free = x - dist_x
        y_free = y - dist_y

        return x_free, y_free, dist_x, dist_y

    def add_distortion(self, x, y, xp, yp, k1, k2, k3, p1, p2, r0):

        # This function takes X, Y (distortion free image coordinates) referenced to the center
        # of the image and outputs them with distortion and principal offset.

        x_update = x
        y_update = y

        iter_num = 0

        XX = x_update
        YY = y_update

        # dist = np.ones((x[:,0].size, x[0,:].size))

        while iter_num <= 100:  # and dist > 0.00001:

            # I = dist > 0.00001 # loop

            # x_update1[I] = 0
            # y_update1[I] = 0

            # x_update2[~I] = 0
            # y_update2[~I] = 0

            XX = x_update
            YY = y_update

            _, _, dist_x, dist_y = self.distortion_free(x_update, y_update, k1, k2, k3, p1, p2, r0)

            # dx = x - x_free
            # dy = y - y_free

            x_update = x + dist_x
            y_update = y + dist_y

            # delt = (dx * dx + dy * dy)
            # dist = np.sqrt(delt)

            iter_num = iter_num + 1

        x_out = XX + xp
        y_out = YY + yp

        return x_out, y_out

