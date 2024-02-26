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

    def image2point(self, R_c_m, r_c_m, img, img_id, line_list, num_of_edge, img_col, img_row, pixel_x, pixel_y, c, k1, k2, k3, p1, p2, r0, xp, yp, all_cloud):

        detection_list = []

        r_c_m_0 = r_c_m[img_id]
        min_x, max_x = r_c_m_0[0] - 50, r_c_m_0[0] + 50
        min_y, max_y = r_c_m_0[1] - 50, r_c_m_0[1] + 50

        section = all_cloud[all_cloud[:, 0] < max_x, :]
        section = section[section[:, 0] > min_x, :]
        section = section[section[:, 1] < max_y, :]
        section = section[section[:, 1] > min_y, :]
        P3 = np.array([np.average(section[:, 0]), np.average(section[:, 1]), np.average(section[:, 2])])
        # avg_x, avg_y, avg_z = np.average(section[:, 0]), np.average(section[:, 1]), np.average(section[:, 2])

        # ray and plane intersection
        covariance_matrix = np.cov(section, rowvar=False)
        e_val, e_vect = np.linalg.eig(covariance_matrix)
        min_eval = np.argmin(e_val)
        normal_vector = e_vect[:, min_eval]
        vector1 = P3 - np.transpose(r_c_m_0)
        u1 = vector1 @ normal_vector

        for i in range(num_of_edge):

            if np.sum(line_list[i]) > 0:

                pt = np.argwhere(line_list[i])
                color = img[line_list[i]]

                if np.average(color[:, 2]) >= 127:
                    color_id = 1 #white
                else:
                    color_id = 0 #yellow

                x_img_distort, y_img_distort = np.floor(pixel_x * (pt[:, 1] - img_col * 0.5 - 0.5)), np.floor(-(pixel_y * (pt[:, 0] - img_row * 0.5 - 0.5)))
                x_img_distort, y_img_distort = x_img_distort - xp, y_img_distort - yp

                x_out, y_out, _, _ = self.distortion_free(x_img_distort, y_img_distort, k1, k2, k3, p1, p2, r0)

                xyc_pt = np.array([x_out, y_out, -c*np.ones(np.size(x_out))])

                if np.size(xyc_pt[:, 0]) > 0:
                    I = R_c_m[img_id] @ xyc_pt
                    u2 = normal_vector @ I
                    u = u1 / u2
                    point = r_c_m_0 + u * I
                    point = np.array([point[0, :], point[1, :], point[2, :], i * np.ones(np.size(point[0, :])), color_id * np.ones(np.size(point[0, :]))])
                    point = np.transpose(point)
                    detection_list.append(point)
                    # detection_list = np.vstack((detection_list, point))

        # detection_list = detection_list[1:, :]

        return detection_list


class lanemarking(object):

    def missing_part(self, line_list, num_of_edge):

        detection_list = np.zeros(num_of_edge, dtype=bool)

        for i in range(num_of_edge):

            num_detection = np.sum(line_list[i])

            if num_detection <= 600:
                detection_list[i] = 1

        return detection_list


    def point2mask(self, m, img_row, img_col, id_of_edge, num_of_edge):

        image_shape = (int(img_row), int(img_col))
        mask_list, edge_list = np.zeros((num_of_edge, int(img_row), int(img_col)), dtype=bool), np.zeros((num_of_edge, int(img_row), int(img_col)), dtype=bool)

        for i in np.unique(m[:, 2]): # right/left/center

            pt = m[m[:, 2] == i, :]
            pt_max, pt_min = pt[pt[:, 0] == np.max(pt[:, 0]), :], pt[pt[:, 0] == np.min(pt[:, 0]), :]
            pt_max = pt_max[0, :]
            pt_min = pt_min[0, :]

            pt1c, pt2c, pt3c, pt4c = int(pt_min[1] - np.ceil(0.04 * pt_min[0])), int(pt_min[1] + np.ceil(0.04 * pt_min[0])), \
                                     int(pt_max[1] - np.ceil(0.08 * pt_max[0])), int(pt_max[1] + np.ceil(0.08 * pt_max[0]))

            pt1 = [int(pt_min[0]), int(pt1c)]
            pt2 = [int(pt_min[0]), int(pt2c)]
            pt3 = [int(pt_max[0]), int(pt3c)]
            pt4 = [int(pt_max[0]), int(pt4c)]

            min_x, max_x = np.min([pt1c, pt2c, pt3c, pt4c]), np.max([pt1c, pt2c, pt3c, pt4c])

            Delt_y = (pt3[0] - pt1[0]) / 6
            line1_r, line1_c = line(int(pt1[0]), int(min_x), int(pt2[0]), int(max_x))
            line2_r, line2_c = line(int(pt3[0] - Delt_y), int(min_x), int(pt4[0] - Delt_y), int(max_x))
            line3_r, line3_c = line(int(pt3[0] - 2 * Delt_y), int(min_x), int(pt4[0] - 2 * Delt_y), int(max_x))
            line4_r, line4_c = line(int(pt3[0] - 3 * Delt_y), int(min_x), int(pt4[0] - 3 * Delt_y), int(max_x))
            line5_r, line5_c = line(int(pt3[0] - 4 * Delt_y), int(min_x), int(pt4[0] - 4 * Delt_y), int(max_x))
            line6_r, line6_c = line(int(pt3[0]), int(min_x), int(pt4[0]), int(pt4[1]))

            edge_temp = np.zeros((int(img_row), int(img_col)), dtype=bool)
            edge_temp[line1_r, line1_c] = 1
            edge_temp[line2_r, line2_c] = 1
            edge_temp[line3_r, line3_c] = 1
            edge_temp[line4_r, line4_c] = 1
            edge_temp[line5_r, line5_c] = 1
            edge_temp[line6_r, line6_c] = 1

            polygon = np.array([pt1, pt2, pt4, pt3])
            mask_temp = polygon2mask(image_shape, polygon)
            I = np.where(id_of_edge == i)
            mask_list[I, :, :] = ~mask_temp
            edge_list[I, :, :] = edge_temp


        return mask_list, edge_list


    def tracking_mask(self, line_list, detection_list, img_row, img_col, num_of_edge):

        image_shape = (int(img_row), int(img_col))
        mask_list = np.zeros((num_of_edge, int(img_row), int(img_col)), dtype=bool)
        edge_list = np.zeros((num_of_edge, int(img_row), int(img_col)), dtype=bool)

        for i in range(num_of_edge):

            edge_temp = np.zeros((int(img_row), int(img_col)), dtype=bool)

            if detection_list[i] == 1:

                pt = np.argwhere(line_list[i])

                if np.sum(pt) == 0:
                    continue

                pt_max = pt[pt[:, 0] == np.max(pt[:, 0]), :]
                pt_max = pt_max[0, :]
                pt_min = pt[pt[:, 0] == np.min(pt[:, 0]), :]
                pt_min = pt_min[0, :]

                length = int(pt_max[0] - pt_min[0] + 1)

                if length > 60:

                    coefficients = np.polyfit(pt[:, 0], pt[:, 1], 1)
                    Delta = -coefficients[1] / coefficients[0]

                    if Delta < (img_row - 1) and Delta > 0:

                        # print(Delta)
                        x1 = 1450 * coefficients[0] + coefficients[1]

                        pt1c = int(x1 - np.ceil(0.04 * 1450))
                        pt2c = int(x1 + np.ceil(0.04 * 1450))

                        pt1 = [int(1450), int(pt1c)]
                        pt2 = [int(1450), int(pt2c)]
                        pt3 = [int(Delta - 25), 0]
                        pt4 = [int(Delta + 25), 0]

                        if (Delta - 25) < 1450:
                            min_y = Delta - 25
                            max_y = 1450
                        else:
                            min_y = 1450
                            max_y = Delta - 25

                        if pt1c < pt2c:
                            min_x = 0
                            max_x = pt2c
                        else:
                            min_x = 0
                            max_x = pt1c

                        Delt_y = (max_y - min_y) / 9
                        line1_r, line1_c = line(int(min_y), int(min_x), int(min_y), int(max_x))
                        line2_r, line2_c = line(int(max_y - Delt_y), int(min_x), int(max_y - Delt_y), int(max_x))
                        line3_r, line3_c = line(int(max_y - 2 * Delt_y), int(min_x), int(max_y - 2 * Delt_y), int(max_x))
                        line4_r, line4_c = line(int(max_y - 3 * Delt_y), int(min_x), int(max_y - 3 * Delt_y), int(max_x))
                        line5_r, line5_c = line(int(max_y - 4 * Delt_y), int(min_x), int(max_y - 4 * Delt_y), int(max_x))
                        line6_r, line6_c = line(int(max_y - 5 * Delt_y), int(min_x), int(max_y - 5 * Delt_y), int(max_x))
                        line7_r, line7_c = line(int(max_y - 6 * Delt_y), int(min_x), int(max_y - 6 * Delt_y), int(max_x))
                        line8_r, line8_c = line(int(max_y - 7 * Delt_y), int(min_x), int(max_y - 7 * Delt_y), int(max_x))
                        line9_r, line9_c = line(int(max_y - 8 * Delt_y), int(min_x), int(max_y - 8 * Delt_y), int(max_x))
                        line10_r, line10_c = line(int(max_y), int(min_x), int(max_y - 50), int(min_x))

                    else:
                        # print(m)
                        # print(m)
                        x1 = 1450 * coefficients[0] + coefficients[1]
                        x2 = (img_row - 1) * coefficients[0] + coefficients[1]

                        pt1c = int(x1 - np.ceil(0.04 * 1450))
                        pt2c = int(x1 + np.ceil(0.04 * 1450))
                        pt3c = int(x2 - np.ceil(0.06 * (img_row - 1)))
                        pt4c = int(x2 + np.ceil(0.06 * (img_row - 1)))

                        pt1 = [int(1450), int(pt1c)]
                        pt2 = [int(1450), int(pt2c)]
                        pt3 = [int(img_row - 1), int(pt3c)]
                        pt4 = [int(img_row - 1), int(pt4c)]

                        if (img_row - 1) < 1450:
                            min_y = img_row - 1
                            max_y = 1450
                        else:
                            min_y = 1450
                            max_y = img_row - 1

                        if pt1c >= 3376:
                            pt1c = 3375

                        if pt2c >= 3376:
                            pt2c = 3375

                        if pt3c >= 3376:
                            pt3c = 3375

                        if pt4c >= 3376:
                            pt4c = 3375

                        min_x = np.min([pt1c, pt2c, pt3c, pt4c])
                        max_x = np.max([pt1c, pt2c, pt3c, pt4c])

                        Delt_y = (max_y - min_y) / 9
                        line1_r, line1_c = line(int(min_y), int(min_x), int(min_y), int(max_x))
                        line2_r, line2_c = line(int(max_y - Delt_y), int(min_x), int(max_y - Delt_y), int(max_x))
                        line3_r, line3_c = line(int(max_y - 2 * Delt_y), int(min_x), int(max_y - 2 * Delt_y), int(max_x))
                        line4_r, line4_c = line(int(max_y - 3 * Delt_y), int(min_x), int(max_y - 3 * Delt_y), int(max_x))
                        line5_r, line5_c = line(int(max_y - 4 * Delt_y), int(min_x), int(max_y - 4 * Delt_y), int(max_x))
                        line6_r, line6_c = line(int(max_y - 5 * Delt_y), int(min_x), int(max_y - 5 * Delt_y), int(max_x))
                        line7_r, line7_c = line(int(max_y - 6 * Delt_y), int(min_x), int(max_y - 6 * Delt_y), int(max_x))
                        line8_r, line8_c = line(int(max_y - 7 * Delt_y), int(min_x), int(max_y - 7 * Delt_y), int(max_x))
                        line9_r, line9_c = line(int(max_y - 8 * Delt_y), int(min_x), int(max_y - 8 * Delt_y), int(max_x))
                        line10_r, line10_c = line(int(max_y), int(min_x), int(max_y), int(max_x))

                    polygon = np.array([pt1, pt2, pt4, pt3])
                    mask_temp = polygon2mask(image_shape, polygon)

                    edge_temp[line1_r, line1_c] = 1
                    edge_temp[line2_r, line2_c] = 1
                    edge_temp[line3_r, line3_c] = 1
                    edge_temp[line4_r, line4_c] = 1
                    edge_temp[line5_r, line5_c] = 1
                    edge_temp[line6_r, line6_c] = 1
                    edge_temp[line7_r, line7_c] = 1
                    edge_temp[line8_r, line8_c] = 1
                    edge_temp[line9_r, line9_c] = 1
                    edge_temp[line10_r, line10_c] = 1

                    mask_list[i, :, :] = ~mask_temp
                    edge_list[i, :, :] = edge_temp

        return mask_list, edge_list

    def equalization(self, img):

        # RGB equalize
        # img_adapteq = color.rgb2gray(exposure.equalize_adapthist(img, clip_limit=0.01))
        # Gray equalize
        img_gray = rgb2gray(denoise_bilateral(img[1450:, :, :], sigma_color=0.05, multichannel=True))
        # img_adapteq2 = exposure.equalize_adapthist(img_gray, clip_limit=0.01)
        img_adapteq3 = equalize_adapthist(img_gray, nbins=100) # clear 100 # bad 256

        return img_adapteq3

    def enlarge_buffer(self, line_list, edges, img_adapteq, img_row, img_col):

        image_shape = (int(img_row), int(img_col))
        edge_temp = np.zeros((int(img_row), int(img_col)), dtype=bool)
        pt0 = line_list
        # thred = 5 * np.std(img_adapteq[pt0])
        pt = np.argwhere(pt0)

        if np.size(pt[:, 0]) <= 20:
            new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
            return new_binary_global

        # RANSAC
        model_robust, inliers = ransac(pt, LineModelND, min_samples=2, residual_threshold=3, max_trials=300)
        pt = pt[inliers]

        pt_max = pt[pt[:, 0] == np.max(pt[:, 0]), :][0, :]
        # pt_max = pt_max[0, :]
        pt_min = pt[pt[:, 0] == np.min(pt[:, 0]), :][0, :]
        # pt_min = pt_min[0, :]

        length = int(pt_max[0] - pt_min[0] + 1)
        # detect_length = np.size(np.unique(pt[:, 0]))
        # detect_length = np.logical_and(detect_length <= pt_max[0], detect_length >= pt_min[0])
        # detect_length = np.sum(detect_length)
        # ratio = detect_length / length

        if length <= 60:
            new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
            return new_binary_global

        coefficients = np.polyfit(pt[:, 0], pt[:, 1], 1)
        Delta = -coefficients[1] / coefficients[0]

        if Delta < (img_row-1) and Delta > 0:

            # print(Delta)
            x1 = 1450 * coefficients[0] + coefficients[1]

            pt1c = int(x1 - np.ceil(0.03 * 1450))
            pt2c = int(x1 + np.ceil(0.03 * 1450))

            pt1 = [int(1450), int(pt1c)]
            pt2 = [int(1450), int(pt2c)]
            pt3 = [int(Delta - 25), 0]
            pt4 = [int(Delta + 25), 0]

            if pt1c >= 3376:
                pt1c = 3375

            if pt2c >= 3376:
                pt2c = 3375

            if pt1c < 0:
                pt1c = 0

            if pt2c < 0:
                pt2c = 0

            if (Delta - 25) < 1450:
                min_y = Delta - 25
                max_y = 1450
            else:
                min_y = 1450
                max_y = Delta - 25

            if pt1c < pt2c:
                min_x = 0
                max_x = pt2c
            else:
                min_x = 0
                max_x = pt1c

            Delt_y = (max_y - min_y) / 9
            line1_r, line1_c = line(int(min_y), int(min_x), int(min_y), int(max_x))
            line2_r, line2_c = line(int(max_y - Delt_y), int(min_x), int(max_y - Delt_y), int(max_x))
            line3_r, line3_c = line(int(max_y - 2 * Delt_y), int(min_x), int(max_y - 2 * Delt_y), int(max_x))
            line4_r, line4_c = line(int(max_y - 3 * Delt_y), int(min_x), int(max_y- 3 * Delt_y), int(max_x))
            line5_r, line5_c = line(int(max_y - 4 * Delt_y), int(min_x), int(max_y - 4 * Delt_y), int(max_x))
            line6_r, line6_c = line(int(max_y - 5 * Delt_y), int(min_x), int(max_y - 5 * Delt_y), int(max_x))
            line7_r, line7_c = line(int(max_y - 6 * Delt_y), int(min_x), int(max_y - 6 * Delt_y), int(max_x))
            line8_r, line8_c = line(int(max_y - 7 * Delt_y), int(min_x), int(max_y - 7 * Delt_y), int(max_x))
            line9_r, line9_c = line(int(max_y - 8 * Delt_y), int(min_x), int(max_y - 8 * Delt_y), int(max_x))
            line10_r, line10_c = line(int(max_y), int(min_x), int(max_y - 50), int(min_x))

        else:
            # print(m)
            # print(m)
            x1 = 1450 * coefficients[0] + coefficients[1]
            x2 = (img_row - 1) * coefficients[0] + coefficients[1]

            pt1c = int(x1 - np.ceil(0.04 * 1450))
            pt2c = int(x1 + np.ceil(0.04 * 1450))
            pt3c = int(x2 - np.ceil(0.05 * (img_row - 1)))
            pt4c = int(x2 + np.ceil(0.05 * (img_row - 1)))

            pt1 = [int(1450), int(pt1c)]
            pt2 = [int(1450), int(pt2c)]
            pt3 = [int(img_row - 1), int(pt3c)]
            pt4 = [int(img_row - 1), int(pt4c)]

            if (img_row - 1) < 1450:
                min_y = img_row - 1
                max_y = 1450
            else:
                min_y = 1450
                max_y = img_row - 1

            if pt1c >= 3376:
                pt1c = 3375

            if pt2c >= 3376:
                pt2c = 3375

            if pt3c >= 3376:
                pt3c = 3375

            if pt4c >= 3376:
                pt4c = 3375

            if pt1c < 0:
                pt1c = 0

            if pt2c < 0:
                pt2c = 0

            if pt3c < 0:
                pt3c = 0

            if pt4c < 0:
                pt4c = 0


            min_x = np.min([pt1c, pt2c, pt3c, pt4c])
            max_x = np.max([pt1c, pt2c, pt3c, pt4c])

            Delt_y = (max_y - min_y) / 9
            line1_r, line1_c = line(int(min_y), int(min_x), int(min_y), int(max_x))
            line2_r, line2_c = line(int(max_y - Delt_y), int(min_x), int(max_y - Delt_y), int(max_x))
            line3_r, line3_c = line(int(max_y - 2 * Delt_y), int(min_x), int(max_y - 2 * Delt_y), int(max_x))
            line4_r, line4_c = line(int(max_y - 3 * Delt_y), int(min_x), int(max_y - 3 * Delt_y), int(max_x))
            line5_r, line5_c = line(int(max_y - 4 * Delt_y), int(min_x), int(max_y - 4 * Delt_y), int(max_x))
            line6_r, line6_c = line(int(max_y - 5 * Delt_y), int(min_x), int(max_y - 5 * Delt_y), int(max_x))
            line7_r, line7_c = line(int(max_y - 6 * Delt_y), int(min_x), int(max_y - 6 * Delt_y), int(max_x))
            line8_r, line8_c = line(int(max_y - 7 * Delt_y), int(min_x), int(max_y - 7 * Delt_y), int(max_x))
            line9_r, line9_c = line(int(max_y - 8 * Delt_y), int(min_x), int(max_y - 8 * Delt_y), int(max_x))
            line10_r, line10_c = line(int(max_y), int(min_x), int(max_y), int(max_x))

        polygon = np.array([pt1, pt2, pt4, pt3])
        mask_temp = polygon2mask(image_shape, polygon)

        edge_temp[line1_r, line1_c] = 1
        edge_temp[line2_r, line2_c] = 1
        edge_temp[line3_r, line3_c] = 1
        edge_temp[line4_r, line4_c] = 1
        edge_temp[line5_r, line5_c] = 1
        edge_temp[line6_r, line6_c] = 1
        edge_temp[line7_r, line7_c] = 1
        edge_temp[line8_r, line8_c] = 1
        edge_temp[line9_r, line9_c] = 1
        edge_temp[line10_r, line10_c] = 1

        # edge_temp[~mask_temp] = 0

        # canny
        img_edge = np.copy(edges)
        img_edge[~mask_temp[1450:, :]] = 0
        # footprint = disk(3)
        # footprint2 = disk(1)
        edges_dilated = dilation(img_edge, disk(3))
        edge_temp[~mask_temp] = 0
        edges_dilated2 = np.copy(edges_dilated)
        edges_dilated[edge_temp[1450:, :]] = 1
        fill_edges = ndi.binary_fill_holes(edges_dilated)
        fill_edges[edges_dilated] = 0
        fill_edges = dilation(fill_edges, disk(1))
        fill_edges[edges_dilated2] = 1

        # Check the area
        fill_edges2 = label(fill_edges)
        regions_data = regionprops_table(
            fill_edges2,
            properties=('label', 'area'),
        )
        table = pd.DataFrame(regions_data)
        # remove small area regions
        area_labels = table['label'] * (table['area'] > 100)
        new_label_area = map_array(
            fill_edges2,
            np.asarray(table['label']),
            np.asarray(area_labels),
        )

        area_labels = new_label_area.astype(bool)
        area_labels[edges_dilated2] = 0

        # RANSAC
        data2 = np.argwhere(area_labels)

        if np.size(data2[:, 0]) > 20:
            model_robust, inliers = ransac(data2, LineModelND, min_samples=2, residual_threshold=15, max_trials=300)
            data3 = data2[inliers]
            # data3 = data3[data3[:, 0] >= 1450]
            data3[:, 0] = data3[:, 0] + 1450
            new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
            if np.size(data3[:, 0]) > 200:
                new_binary_global[data3[:, 0], data3[:, 1]] = 1

                # Check the solidity
                area_labels2 = label(new_binary_global)
                regions_data = regionprops_table(
                    area_labels2,
                    properties=('label', 'area', 'solidity'),
                )
                table = pd.DataFrame(regions_data)
                # remove small area regions
                sol_labels = table['label'] * (table['area'] > 2000) + table['label'] * (table['area'] > 200) * (table['solidity'] > 0.8)
                new_label_sol = map_array(
                    area_labels2,
                    np.asarray(table['label']),
                    np.asarray(sol_labels),
                )
                new_binary_global = new_label_sol.astype(bool)

        else:
            new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)

        return new_binary_global

    def canny_thresholding(self, img, img_row, img_col, mask_list, edge_list, num_of_edge):

        line_list = np.zeros((num_of_edge, int(img_row), int(img_col)), dtype=bool)
        # RGB equalize
        # img_adapteq = exposure.equalize_adapthist(img, clip_limit=0.01)
        # Gray equalize

        img_adapteq3 = self.equalization(img)
        # img_gray = rgb2gray(equalize_adapthist(img[1450:, :, :], clip_limit=0.01))
        # img_adapteq2 = exposure.equalize_adapthist(img_gray, clip_limit=0.01)
        # img_adapteq3 = denoise_bilateral(equalize_adapthist(img_gray, clip_limit=0.01), sigma_color=0.03)
        # img_adapteq2 = exposure.equalize_adapthist(img_adapteq, clip_limit=0.1)

        edges = feature.canny(img_adapteq3, sigma=5, low_threshold=0.03, high_threshold=0.1)


        for i in range(num_of_edge): # right/left/center # need to change

            if np.sum(mask_list[i]) == 0:
                continue

            # canny
            img_edge = np.copy(edges)
            img_edge[mask_list[i, 1450:, :]] = 0
            footprint = disk(3)
            footprint2 = disk(1)
            edges_dilated = dilation(img_edge, footprint)
            edge_temp = edge_list[i, 1450:, :]
            edge_temp[mask_list[i, 1450:, :]] = 0
            edges_dilated2 = np.copy(edges_dilated)
            edges_dilated[edge_temp] = 1
            fill_edges = ndi.binary_fill_holes(edges_dilated)
            fill_edges[edges_dilated] = 0
            fill_edges = dilation(fill_edges, footprint2)
            fill_edges[edges_dilated2] = 1

            # Check the area
            fill_edges2 = label(fill_edges)
            regions_data = regionprops_table(
                fill_edges2,
                properties=('label', 'area'),
            )
            table = pd.DataFrame(regions_data)
            # remove small area regions
            area_labels = table['label'] * (table['area'] > 100)
            new_label_area = map_array(
                fill_edges2,
                np.asarray(table['label']),
                np.asarray(area_labels),
            )

            area_labels = new_label_area.astype(bool)
            area_labels[edges_dilated2] = 0

            # RANSAC
            data2 = np.argwhere(area_labels)

            if np.size(data2[:, 0]) > 20:
                _, inliers = ransac(data2, LineModelND, min_samples=2, residual_threshold=15, max_trials=300)
                data3 = data2[inliers]
                data3[:, 0] = data3[:, 0] + 1450
                # data3 = data3[data3[:, 0] >= 1450]
                new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
                if np.size(data3[:, 0]) > 200:
                    new_binary_global[data3[:, 0], data3[:, 1]] = 1
                    # Check the solidity
                    area_labels2 = label(new_binary_global)
                    regions_data = regionprops_table(
                        area_labels2,
                        properties=('label', 'area', 'solidity'),
                    )
                    table = pd.DataFrame(regions_data)
                    # remove small area regions
                    sol_labels = table['label'] * (table['area'] > 2000) + table['label'] * (table['area'] > 200) * (table['solidity'] > 0.8)
                    new_label_sol = map_array(
                        area_labels2,
                        np.asarray(table['label']),
                        np.asarray(sol_labels),
                    )
                    new_binary_global = new_label_sol.astype(bool)
                    new_binary_global = self.enlarge_buffer(new_binary_global,
                    edges, img_adapteq3, img_row, img_col)


            else:
                new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)

            line_list[i, :, :] = new_binary_global

        return img_adapteq3, line_list

    def tracking_canny_thresholding(self, img_adapteq, img_row, img_col, mask_list, edge_list, line_list, num_of_edge):

        img_adapteq3 = img_adapteq

        edges = feature.canny(img_adapteq3, sigma=5, low_threshold=0.03, high_threshold=0.1)

        for i in range(num_of_edge): # right/left/center # need to change

            if np.sum(mask_list[i]) == 0:
                continue
            # canny
            img_edge = np.copy(edges)
            img_edge[mask_list[i, 1450:, :]] = 0
            # footprint = disk(3)
            # footprint2 = disk(1)
            edges_dilated = dilation(img_edge, disk(3))
            edge_temp = edge_list[i, 1450:, :]
            edge_temp[mask_list[i, 1450:, :]] = 0
            edges_dilated2 = np.copy(edges_dilated)
            edges_dilated[edge_temp] = 1
            fill_edges = ndi.binary_fill_holes(edges_dilated)
            fill_edges[edges_dilated] = 0
            fill_edges = dilation(fill_edges, disk(1))
            fill_edges[edges_dilated2] = 1

            # Check the area
            fill_edges2 = label(fill_edges)
            regions_data = regionprops_table(
                fill_edges2,
                properties=('label', 'area'),
            )
            table = pd.DataFrame(regions_data)
            # remove small area regions
            area_labels = table['label'] * (table['area'] > 100)
            new_label_area = map_array(
                fill_edges2,
                np.asarray(table['label']),
                np.asarray(area_labels),
            )

            area_labels = new_label_area.astype(bool)
            area_labels[edges_dilated2] = 0

            # RANSAC
            data2 = np.argwhere(area_labels)

            if np.size(data2[:, 0]) > 20:
                model_robust, inliers = ransac(data2, LineModelND, min_samples=2, residual_threshold=15, max_trials=300)
                data3 = data2[inliers]
                # data3 = data3[data3[:, 0] >= 1450]
                data3[:, 0] = data3[:, 0] + 1450
                new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
                if np.size(data3[:, 0]) > 200:
                    new_binary_global[data3[:, 0], data3[:, 1]] = 1
                    # Check the solidity
                    area_labels2 = label(new_binary_global)
                    regions_data = regionprops_table(
                        area_labels2,
                        properties=('label', 'area', 'solidity'),
                    )
                    table = pd.DataFrame(regions_data)
                    # remove small area regions
                    sol_labels = table['label'] * (table['area'] > 200) * (table['solidity'] > 0.8)
                    new_label_sol = map_array(
                        area_labels2,
                        np.asarray(table['label']),
                        np.asarray(sol_labels),
                    )
                    new_binary_global = new_label_sol.astype(bool)
                    # new_binary_global = self.enlarge_buffer(new_binary_global,
                    # edges, img_adapteq3, img_row, img_col)
            else:
                new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)

            if np.any(new_binary_global):
                line_list[i, :, :] = new_binary_global

        return img_adapteq3, line_list

    # def color_thresholding(self, m, img, img_row, img_col, mask_list, percentile, id_of_edge, num_of_edge):
    #
    #     line_list = np.zeros((num_of_edge, int(img_row), int(img_col)), dtype=bool)  # right/left/center
    #     # dist_list = np.zeros((3, 1))
    #     # id = id_of_edge
    #     # RGB equalize
    #     img_adapteq = exposure.equalize_adapthist(img, clip_limit=0.01)
    #     # Gray equalize
    #     img_gray = color.rgb2gray(img_adapteq)
    #     img_adapteq2 = exposure.equalize_adapthist(img_gray, clip_limit=0.01)
    #
    #     for i in range(num_of_edge): # right/left/center # need to change
    #
    #         if np.all(mask_list[i]):
    #             continue
    #         # 95%
    #         img_adapteq3 = np.copy(img_adapteq2)
    #         img_adapteq3[mask_list[i]] = np.nan
    #         data = img_adapteq3[~np.isnan(img_adapteq3)]
    #         cdf = exposure.cumulative_distribution(data)
    #         I = np.abs(cdf[0] - percentile) == np.min(np.abs(cdf[0] - percentile))
    #         threshold = cdf[1]
    #         threshold = threshold[I]
    #         if np.size(threshold) > 1:
    #             threshold = threshold[0]
    #         binary_global = img_adapteq3 > threshold
    #
    #         # Check the area
    #         binary_global_label = label(binary_global)
    #         # regions = regionprops(binary_global_label)
    #         regions_data = regionprops_table(
    #             binary_global_label,
    #             properties=('label', 'area'),
    #         )
    #         table = pd.DataFrame(regions_data)
    #         # remove small area regions
    #         area_labels = table['label'] * (table['area'] > 50)
    #         new_label_area = map_array(
    #             binary_global_label,
    #             np.asarray(table['label']),
    #             np.asarray(area_labels),
    #         )
    #         area_labels = new_label_area.astype(bool)
    #
    #         # RANSAC for each region
    #         area_labels2 = label(area_labels)
    #         regions = regionprops(area_labels2)
    #         data2_filtered = np.empty([1, 2])
    #
    #         for props in regions:
    #             data2 = props.coords
    #             model_robust, inliers = ransac(data2, LineModelND, min_samples=2,
    #                                            residual_threshold=5, max_trials=100)
    #             # np.concatenate([data2_filtered, data2[inliers]], axis=0).shape
    #             data2_filtered = np.vstack((data2_filtered, data2[inliers]))
    #
    #         data2_filtered = np.delete(data2_filtered, 0, axis=0)
    #         data2_filtered = data2_filtered.astype(int)
    #         binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
    #         binary_global[data2_filtered[:, 0], data2_filtered[:, 1]] = 1
    #
    #         # check minor axis and orientation with centerline
    #         binary_global_label = label(binary_global)
    #         regions_data = regionprops_table(
    #             binary_global_label,
    #             properties=('label', 'centroid', 'minor_axis_length'),
    #         )
    #         table = pd.DataFrame(regions_data)
    #         axis_labels = table['label'] * (table['minor_axis_length'] < (table['centroid-1'] / 2395 * 60))
    #         new_label_axis = map_array(
    #             binary_global_label,
    #             np.asarray(table['label']),
    #             np.asarray(axis_labels),
    #         )
    #         axis_labels = new_label_axis.astype(bool)
    #         new_binary_global = axis_labels
    #
    #         # data2 = np.argwhere(mask)
    #         # RANSAC
    #         # data2 = np.argwhere(new_binary_global)
    #         # model_robust, inliers = ransac(data2, LineModelND, min_samples=2,
    #         #                                 residual_threshold=1, max_trials=1000)
    #         # data3 = data2[inliers]
    #
    #         # check with centerline (mode distance and number of pixels)
    #         # pt = m[m[:, 2] == id[i], :]
    #         # pt_max = pt[pt[:, 0] == np.max(pt[:, 0]), :]
    #         # pt_max = pt_max[0, :]
    #         # pt_max = np.array([pt_max[0], pt_max[1]])
    #         # pt_min = pt[pt[:, 0] == np.min(pt[:, 0]), :]
    #         # pt_min = pt_min[0, :]
    #         # pt_min = np.array([pt_min[0], pt_min[1]])
    #         #
    #         # delta_1 = pt_min - data2
    #         # # delta_1 = np.transpose(delta_1)
    #         # delta_2 = pt_max - pt_min
    #         # delta_2 = np.tile(delta_2, (np.size(delta_1[:, 1]), 1))
    #         # delta_3 = np.array([np.cross(delta_2, delta_1)])
    #         # norm_1 = np.array([np.linalg.norm(delta_3, axis=0)])
    #         # norm_2 = np.array([np.linalg.norm(delta_2, axis=1)])
    #         # dist = norm_1 / norm_2
    #         # int_dist = dist.astype(int)
    #         # dist = stats.mode(int_dist[0, :])
    #         # dist = dist[0]
    #         # mask = int_dist == dist
    #         # in_data2 = data2[mask[0, :], :]
    #         #
    #         # if np.size(in_data2[:, 0]) >= 50:
    #         #     # RANSAC
    #         #     model_robust, inliers = ransac(in_data2, LineModelND, min_samples=2,
    #         #                                    residual_threshold=1, max_trials=1000)
    #         #     in_data3 = in_data2[inliers]
    #         #     new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
    #         #     new_binary_global[in_data3[:, 0], in_data3[:, 1]] = 1
    #         # else:
    #         #     new_binary_global = np.zeros((int(img_row), int(img_col)), dtype=bool)
    #         line_list[i, :, :] = new_binary_global
    #
    #     return line_list