# VisualHull Overview
This is the final term homework for Data Structure course in Tsinghua Univ.

The goal of this work is to improve the performance of a algorithm which constructs a points cloud from a series of bmp(png) pictures from different directions.

The original implementation of the function is based on a 3d vector. The algorithm will travarse the whole voxel and determine if the point is in any of the pictures. An effective way to improve the algorithm is to reduce points compared, and I planed to use octree to ahchive this. The spirit of the algorithm is the same as Shell Sort, "coarse to fine".


# Result 

20170115: The project had passed the defense of the final term homework with top mark.



Before the improvement:

time for initialization: 0.01seconds

time for loading matrix: 0.046seconds

time for loading images: 0.343seconds

time for getting model: 5.418seconds

time for getting surface: 0.057seconds

time for saving model without normal: 0.247seconds

time for saving model with normal: 0.81seconds

time for poinsson reconstruction 1.542seconds

total time: 8.476seconds



After the improvement:

time for initialization: 0.004seconds

time for loading matrix: 0seconds

time for loading images: 0.11seconds

time for getting model: 0.004seconds

time for getting surface: 0.305seconds

time for saving model without normal: 0.149seconds

time for saving model with normal: 0.755seconds

time for poinsson reconstruction 1.514seconds

total time: 2.845seconds


# Contents of files and dictionarys
octree: Basic implementation of octree. The source file is adopted for the actuall project.

test: For test as it shows.

VisualHull: Key source files, including final implementation of octree(octree.h), and the framework of the final program(main.cpp)

VisualHull_Full: All files for a VC solution, including CV lib, 2 sets of test data, output data and so on.

