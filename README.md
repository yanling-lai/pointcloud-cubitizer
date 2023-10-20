# pointcloud-cubitizer

This program split point cloud into cubes with user defined cube size and overlap percentage between cubes.

### Input

Program takes in a text file that contains all of the paths to the source point clouds, and a text file contains their corresponding label file paths. Program process point cloud files in text file format.\
Note: Current version only process point clouds that have corresponding label files.

### Output

Program outputs each of the cube as text files.

### Usage

python process_pointclouds.py -pf [file path] -lf [file path] -d [dir path] -cs [cubic_size_x cubic_size_y cubic_size_z] -o [overlap_x overlap_y overlap_z]\
\
Required:\
-pf: File path to text file that contains paths to source point cloud files.\
-lf: File path to text file that contains paths to source point cloud label files. (shold be in same order as source point cloud file paths)\
-d: Directory to store output.\
-cs: Cubic size in direction x, y, and z.\
-o Overlap percentage of cubics in direction x, y, and z.\
-k: Fixed number of points in each cube, discard cube when not sufficient, trim when oversize. Set to 0 to keep every cube that contains at least 1 point without any trimming.

Optional:\
-b: Toggle on to store boundaries of each cubic in each output file's first two rows.

### Example

Cubitize 3 point clouds with cubic of size 10 and 25% opverlap, and keep every cube that contains point:\
python process_pointclouds.py -pf pointcloud_file_list.txt -lf pointcloud_label_file_list.txt -d Output/ -cs 10 10 10 -o 0.25 0.25 0.25 -k 0\
\
pointcloud_file_list.txt contains:\
pointcloud1.txt\
pointcloud2.txt\
pointcloud3.txt\
\
pointcloud_label_file_list.txt contains:\
pointcloud1_label.txt\
pointcloud2_label.txt\
pointcloud3_label.txt\
\
Output:\
1.txt\
2.txt\
5.txt\
...