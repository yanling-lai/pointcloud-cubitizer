""" 
input:
    - file path
    - cubic size (x, y, z)
    - overlap (x, y, z) (in percentage of cubic size, e.g. (0.25, 0.25, 0.25))
steps:
    >> calculate number of bins in each coordinate, and get number of total cubic bins
    * Note: index starts with 0
    >> incoming points
        1. get point coordinate, center to zero
        2. find ranges of which the point lies for each of the coordinate (e.g. x:[2, 3, 4], y:[4, 5], z:[6])
        4. through combinations of list from 2, get full list of cubic bins that contains the point 
            (i.e. (2, 4, 6), (2, 5, 6), (3, 4, 6), (3, 5, 6), (4, 4, 6), (4, 5, 6))
        3. append point to the bins (use locating as x_i + y_i*cube_num_x_axis + z_i*cube_num_xy_plane)
"""

import numpy as np
import argparse

class pointcloud_cubitizer:
    """ class that stores point cloud information and execute cubitization """
    def __init__(self, point_file_path, label_file_path, cubic_size, overlap, min_num_of_point):
        self.min_num_of_point = min_num_of_point
        self.point_file_path = point_file_path
        self.label_file_path = label_file_path
        print("loading files...")
        self.point_cloud = np.loadtxt(self.point_file_path)
        self.labels = np.loadtxt(self.label_file_path)
        print("done loading, initializing some parameters...")
        self.cubic_size = np.array(cubic_size)
        self.overlap = np.array(overlap)
        self.slide_step = (np.array([1, 1, 1]) - self.overlap) * self.cubic_size
        self.num_of_points = self.point_cloud.shape[0]

        self.min_xyz = np.min(self.point_cloud, axis=0)
        self.max_xyz = np.max(self.point_cloud, axis=0)
        self.range_xyz = self.max_xyz - self.min_xyz
        self.num_of_bins_xyz = np.ceil((self.range_xyz - self.cubic_size)/self.slide_step) + np.array([1, 1, 1])
        self.num_of_bins_xyz = self.num_of_bins_xyz.astype(int)
        self.num_of_xbins = self.num_of_bins_xyz[0]
        self.num_of_xybins = self.num_of_bins_xyz[0]*self.num_of_bins_xyz[1]
        self.total_cubic_num = self.num_of_bins_xyz[0]*self.num_of_bins_xyz[1]*self.num_of_bins_xyz[2]
        self.cubic_boxes = [[] for _ in range(self.total_cubic_num)]

    def throw(self, point_label):
        point = point_label[0:3] - self.min_xyz
        start_box = np.maximum(np.ceil((point - self.cubic_size)/self.slide_step), np.array([0, 0, 0]))
        end_box = np.minimum(np.floor(point/self.slide_step), self.num_of_bins_xyz-np.array([1, 1, 1]))

        xbins_with_point = np.arange(start_box[0], end_box[0]+1, 1)
        xbins_with_point = xbins_with_point.astype(int)
        ybins_with_point = np.arange(start_box[1], end_box[1]+1, 1)
        ybins_with_point = ybins_with_point.astype(int)
        zbins_with_point = np.arange(start_box[2], end_box[2]+1, 1)
        zbins_with_point = zbins_with_point.astype(int)

        for i in range(xbins_with_point.shape[0]):
            for j in range(ybins_with_point.shape[0]):
                for k in range(zbins_with_point.shape[0]):
                    self.cubic_boxes[xbins_with_point[i]+ybins_with_point[j]*self.num_of_xbins+zbins_with_point[k]*self.num_of_xybins].append([point_label[0], point_label[1], point_label[2], point_label[3]])

    def cubitize(self):
        np.apply_along_axis(self.throw, 1, np.hstack((self.point_cloud, self.labels.reshape((-1, 1)))))

    def calculate_boundary(self, i):
        z = int(i/self.num_of_xybins)
        y = int((i-z*self.num_of_xybins)/self.num_of_xbins)
        x = int(i-z*self.num_of_xybins-y*self.num_of_xbins)
        box_i_coordinate = np.array([x, y, z])

        min_bd = box_i_coordinate*self.slide_step + self.min_xyz
        max_bd = min_bd + self.cubic_size

        box_i_boundary = np.vstack((min_bd, max_bd))

        return box_i_boundary

    def write_to_disk(self, target_dir, start_index, store_boundary):
        if (store_boundary):
            if (self.min_num_of_point == 0):
                for i in range(len(self.cubic_boxes)):
                    points = np.asarray(self.cubic_boxes[i])
                    if (points.shape[0] <= self.min_num_of_point):
                        continue
                    box_i_boundary = self.calculate_boundary(i)
                    sv = np.vstack((box_i_boundary, points))
                    np.savetxt(target_dir + str(i+start_index) + '.txt', sv, fmt='%.5f')
                return

            for i in range(len(self.cubic_boxes)):
                points = np.asarray(self.cubic_boxes[i])
                if (points.shape[0] <= self.min_num_of_point):
                    continue
                box_i_boundary = self.calculate_boundary(i)
                points = points[np.random.choice(points.shape[0], self.min_num_of_point, replace=False)]
                sv = np.vstack((box_i_boundary, points))
                np.savetxt(target_dir + str(i+start_index) + '.txt', sv, fmt='%.5f')
            return

        if (self.min_num_of_point == 0):
            for i in range(len(self.cubic_boxes)):
                points = np.asarray(self.cubic_boxes[i])
                if (points.shape[0] <= self.min_num_of_point):
                    continue
                np.savetxt(target_dir + str(i+start_index) + '.txt', points, fmt='%.5f')
            return
        
        for i in range(len(self.cubic_boxes)):
            points = np.asarray(self.cubic_boxes[i])
            if (points.shape[0] <= self.min_num_of_point):
                continue
            points = points[np.random.choice(points.shape[0], self.min_num_of_point, replace=False)]
            np.savetxt(target_dir + str(i+start_index) + '.txt', points, fmt='%.5f')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                    prog='Point Cloud Cubitizer',
                    description='This program cubitize point cloud with sliding cubes')
    parser.add_argument('-pf', '--point_file_path', type=str,
                    help='file path to source point cloud', required=True)
    parser.add_argument('-lf', '--label_file_path', type=str,
                    help='file path to source point cloud label', required=True)
    parser.add_argument('-d', '--target_dir', type=str,
                    help='directory to store output', required=True)
    parser.add_argument('-cs', '--cubic_size', nargs='+', type=int,
                    help='cubic size x y z', required=True)
    parser.add_argument('-o', '--overlap', nargs='+', type=float,
                    help='overlap percentage x y z', required=True)
    parser.add_argument('-k', '--min_num_of_point', type=int,
                    help='lower bound of number of points in each cube, discard cube when not sufficient, trim when oversized', required=True)
    
    args = parser.parse_args()
    cubitizer = pointcloud_cubitizer(args.point_file_path, args.label_file_path, args.cubic_size, args.overlap, args.min_num_of_point)
    print(cubitizer.num_of_points)
    print(cubitizer.num_of_bins_xyz)
    print(cubitizer.total_cubic_num)
    print(cubitizer.slide_step)
    print("start cubitizing...")
    cubitizer.cubitize()
    print("total boxes:")
    print(len(cubitizer.cubic_boxes))
    cubitizer.write_to_disk(args.target_dir)

    