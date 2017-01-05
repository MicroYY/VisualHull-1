# VisualHull
This is the final term homework for Data Structure course in Tsinghua Univ.
The goal of this work is to improve the performance of a algorithm which constructs a points cloud from a series of bmp pictures from different directions.
The original implementation of the function is based on a 3d vector. The algorithm will travarse the whole voxel and determine if the point is in any of the pictures. An effective way to improve the algorithm is to reduce points compared, and I planed to use octree to ahchive this.
First I shall test an octree. Then I'll combine it to the whole project.
