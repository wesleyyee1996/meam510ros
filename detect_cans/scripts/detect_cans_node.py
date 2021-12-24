#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

class DetectCans:
    """
    Node to detect cans. We didn't end up using this
    """
    def __init__(self):
        self.map_width = 1600
        self.map_height = 1600
        self.x_center = 800
        self.y_center = 800
        self.r = 36
        self.accumulate = [[0 for x in range(self.map_width)] for y in range(self.map_height)]
        self.raw_x_pts = []
        self.raw_y_pts = []
        self.data = []
        self.clusters = []
        self.good_clusters = []
        self.max_coord = [0,0]
        self.max_val = 0
        self.diff_tolerance = 12
        self.min_cluster_len = 4


    def load_new_lidar_data(self, new_data):
        self.data = new_data

    def convertPolarToCartesian(self,theta, dist, x_c, y_c):
        out = []
        out.append(x_c + math.cos(theta*math.pi/180)*dist)
        out.append(y_c + math.sin(theta*math.pi/180)*dist)
        return out

    def make_raw_x_y_pts(self):
        for i in range(360):
            point = self.convertPolarToCartesian(i, self.data[i], self.x_center, self.y_center)
            self.raw_x_pts.append(point[0])
            self.raw_y_pts.append(point[1])

    def plot_raw_data(self):
        fig = plt.figure()
        plt.scatter(self.raw_x_pts, self.raw_y_pts, s= 1)
        plt.scatter(self.x_center, self.y_center, s = 2, c='r')
        plt.xlim(0,1600)
        plt.ylim(0,1600)
        plt.show()

    def cluster_segments(self):
        """
        Finds clusters of points. In order to pass this function, points next to each other must not exceed
        an absolute difference in length of the diff_tolerance
        """
        i = 0
        while i < len(self.data):
            j = i+1
            while j < len(self.data) and (abs(self.data[j] - self.data[j-1]) < self.diff_tolerance):
                j += 1
            j -= 1
            if j-i > self.min_cluster_len:
                self.clusters.append([i,j])
            i = j+1

    def validate_arc(self):
        """
        A higher level validation function which sees if a given cluster matches
        up with the general shape of a semicircular object. Just filters so the detect
        function doesn't have to run for all clusters
        """
        for cluster in self.clusters:
            left_idx = cluster[1]
            right_idx = cluster[0]
            mid_idx = (right_idx-left_idx)//2+left_idx
            angle = -(mid_idx - 90)/57.2958
            left = self.convertPolarToCartesian(left_idx, self.data[left_idx], 0, 0)
            mid =  self.convertPolarToCartesian(mid_idx, self.data[mid_idx], 0, 0)
            right =  self.convertPolarToCartesian(right_idx, self.data[right_idx], 0, 0)
            left = self.rotate(left[0], left[1], angle)
            mid = self.rotate(mid[0], mid[1], angle)
            right = self.rotate(right[0], right[1], angle)
            diameter = self.distance(left[0], left[1], right[0], right[1])
            mid_pt = [(left[0]+right[0])/2, (left[1]+right[1])/2]
            dist = self.distance(mid_pt[0], mid_pt[1], mid[0], mid[1])
            if (0.15*diameter < dist < 0.25*diameter):
                self.good_clusters.append(cluster)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2+(y1-y2)**2)

    def rotate(self, x, y, angle):
        """
        Rotate the given point by a certain angle in radians
        """
        return [x*math.cos(angle)-y*math.sin(angle), x*math.sin(angle)+y*math.cos(angle)]

    def detect(self):
        
        # after finding the good clusters, then use least squares to determine if the cluster
        # is actually a can or not
        for cluster in self.good_clusters:

            x = np.array(self.raw_x_pts[cluster[0]:cluster[1]])
            y = np.array(self.raw_y_pts[cluster[0]:cluster[1]])
            # print(x,y)
            A = np.stack((2*x, 2*y, -1*np.ones((len(x)), dtype = int)), axis = 1)
            # print(cluster, x, y)
            b = np.array(np.power(x,2)+np.power(y,2))
            # print(A, b)
            A_transpose = A.transpose()
            ATA = A_transpose.dot(A)
            ATb = A_transpose.dot(b)
            x_est = (np.linalg.inv(ATA)).dot(ATb)

            x_0 = x_est[0]
            y_0 = x_est[1]
            r = math.sqrt(x_0**2 + y_0**2 - x_est[2])
            print(cluster[0], cluster[1], x_0, y_0, r)
