# Visual Hull
## Overview
This is the final term homework for Data Structure course in Tsinghua Univ.

The goal of this work is to improve the performance of a algorithm which constructs a points cloud from a series of bmp(png) pictures from different directions.

The original implementation of the function is based on a 3d vector. The algorithm will travarse the whole voxel and determine if the point is in any of the pictures. An effective way to improve the algorithm is to reduce points compared, and I planed to use octree to ahchive this. The spirit of the algorithm is the same as Shell Sort, "coarse to fine".


## Results

20170115: The project had passed the defense of the final term homework with top mark.



Before the improvement(copied from output file):

>time for initialization: 0.01seconds<br>
>time for loading matrix: 0.046seconds<br>
>time for loading images: 0.343seconds<br>
>time for getting model: 5.418seconds<br>
>time for getting surface: 0.057seconds<br>
>time for saving model without normal: 0.247seconds<br>
>time for saving model with normal: 0.81seconds<br>
>time for poinsson reconstruction 1.542seconds<br>
>total time: 8.476seconds<br>


After the improvement:

>time for initialization: 0.004seconds<br>
>time for loading matrix: 0seconds<br>
>time for loading images: 0.11seconds<br>
>time for getting model: 0.004seconds<br>
>time for getting surface: 0.305seconds<br>
>time for saving model without normal: 0.149seconds<br>
>time for saving model with normal: 0.755seconds<br>
>time for poinsson reconstruction 1.514seconds<br>
>total time: 2.845seconds<br>


## Contents of Files and Dictionarys


- VisualHull: Key source files, including final implementation of octree(octree.h), and the framework of the final program(main.cpp)

- VisualHull_Full: All files for a VC solution, including CV lib, 2 sets of test data, output data and so on.

- test: For test as its name shows.

- defense.pptx: the ppt of the defense to the advisor for this project. The file's in Chinese.

## Future Work?

- add texture?
- the parameters of the cameras is unknown?
- **GPU acceleated reconstruction?**
