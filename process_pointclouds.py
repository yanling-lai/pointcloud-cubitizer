import pointcloud_cubitizer as pcc
import argparse
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                    prog='Point Cloud Cubitizer',
                    description='This program cubitize point cloud with sliding cubes')
    parser.add_argument('-pf', '--point_file_paths', type=str,
                    help='file paths to text files that contain source point cloud paths', required=True)
    parser.add_argument('-lf', '--label_file_paths', type=str,
                    help='file paths to text files that contain source point cloud label paths', required=True)
    parser.add_argument('-d', '--target_dir', type=str,
                    help='directory to store output', required=True)
    parser.add_argument('-cs', '--cubic_size', nargs='+', type=int,
                    help='cubic size x y z', required=True)
    parser.add_argument('-o', '--overlap', nargs='+', type=float,
                    help='overlap percentage x y z', required=True)
    parser.add_argument('-k', '--min_num_of_point', type=int,
                    help='lower bound of number of points in each cube, discard cube when not sufficient, trim when oversized', required=True)
    parser.add_argument('-b', '--store_boundary', type=bool, default=False,
                    help='toggle to store boundaries of each box in each output file\'s first two rows')
    
    args = parser.parse_args()

    cubic_size = args.cubic_size
    overlap = args.overlap
    min_num_of_point = args.min_num_of_point
    target_dir = args.target_dir
    store_boundary = args.store_boundary

    # point file paths
    pf = open(args.point_file_paths, "r")
    point_file_paths_r = pf.read() 
    point_file_paths = point_file_paths_r.split('\n')
    if point_file_paths[-1] == '':
        del point_file_paths[-1]
    pf.close()

    # label file paths
    pf = open(args.label_file_paths, "r") 
    label_file_paths_r = pf.read() 
    label_file_paths = label_file_paths_r.split('\n')
    if label_file_paths[-1] == '':
        del label_file_paths[-1]
    pf.close()

    start_index = 0
    for i in range(len(point_file_paths)):
        print("initialzing file: " + point_file_paths[i] + " and its label file: " + label_file_paths[i])
        cubitizer = pcc.pointcloud_cubitizer(point_file_paths[i], label_file_paths[i], cubic_size, overlap, min_num_of_point)
        print("total num of points: " + str(cubitizer.num_of_points))
        print("num of bins [#x, #y, #z]" + str(cubitizer.num_of_bins_xyz))
        print("slide step: " + str(cubitizer.slide_step))
        print("total empty boxes succesfully established: " + str(len(cubitizer.cubic_boxes)))

        print("done initializing, start cubitizing...")
        cubitizer.cubitize()

        print("done cubitizing, writing to disk, start_index: " + str(start_index))
        cubitizer.write_to_disk(args.target_dir, start_index, args.store_boundary)
        start_index += len(cubitizer.cubic_boxes)