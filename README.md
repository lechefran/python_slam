## Python SLAM    
This is a simple, evolving simultaneous localization and mapping program.
This project is currently being developed using Python 3.x on Ubuntu 18.04 operating system. The program is now currently in the 3D map display
and graph optimization stage with well coordinated and improved 3D map display as a future goal of the project.

## Program Usage  
After cloning this repository, run the program using
```
./slam.py (additional info: -t or -f) video_name_here
python3 slam.py (additional info: -t or -f) video_name_here
```
## Required Libraries
This program is being written in Python3 and uses multiple libraries that have to be installed by the user separately.  
Here is a list of the current fundamental Python libaries necessary to the project:  

**OpenCV**  
OpenCV is a library that contains numerous computer vision algorithms. Check them out at [OpenCV](https://docs.opencv.org/3.4/d1/dfb/intro.html).  
To install, use the command `python3 -m pip install opencv-python`.

**PySDL2**  
PySDL2 is a Python wrapper for the SDL2 library. Check them out at [PySDL2](https://pysdl2.readthedocs.io/en/rel_0_9_6/index.html).  
To install, use the command `python3 -m pip install pysdl2`.

**Scikit-image**
Scikit-image (skimage) is a collection of image processing algorithms. Check them out at [Scikit-image](https://scikit-image.org/).  
To install, use the command `python3 -m pip install scikit-image`.

## Additional Requirements
Here is a list of additional requirements that are used for this project:

**g2opy**
This is a python binding of graph optimization C++ g2o. Check them out at [g2opy](https://github.com/uoip/g2opy).
You can follow the installation steps using the provided link. Note that there may be problems that occur when following the installation process
provided. This can be fixed by import the system-specific parameters and functions library using *import sys* and appending the directory path of g2o to the program.
Another potential fix to any build problems (especially if only private modules are found) is to specify the python version when building by using `cmake -DPYBIND11_PYTHON_VERSION=3.6 ..`
and `make -j8` when building the dependency for this program.

**Pangolin**  
Pangolin is a lightweight library for managing the OpenGL display/interaction and abstracting video input. The version being used for this project
is the python binding for the Pangolin project. Check it out at [Pangolin](https://github.com/uoip/pangolin). Follow the installation instructions
on the GitHub page. If there are any problems with building, please consult the build steps for g2o found above.

Note that these additional pybindings were built using Ubuntu 16.04 due to the fact that they build properly and reliably in that distribution version then in any other version.
Build in another version at own user discretion. It is also recommended that you place all these additional repositories in a separate directory to keep you repository clean.
Build all of the additional repositories in that separate directory.

## Current Issues  
There are currently some issues in regards to the performance of the optimizer. This may have be an issue that has blindsided the project and will be investigated.

## Current Updates
2/19/20  
Refactoring of code. Added culling to program and some cases to prevent runtime issues.

2/4/20  
Refactoring and adding new things to the code. New classes and functions. Added a basic optimizer for the 3D display.
General code cleanup to come in the near future.

1/23/20
Finally got a working version of the 3D map display to work. Still a bit funny acting but works properly and even works
as a thread.

1/22/20  
Successfully built both g2opy and pangolin python bindings. Program refactoring now stores point indices.
Indices are not stored and program now makes use of points as objects. Initial version of the 3D map
display for the program coming soon.

12/20/19
General code cleanup especially in frame.py. Added major changes to extraction method: now based
on image frames versus having an extractor object do all of the work.

8/15/19  
Inclusion of rotation and transformation matrix information to the program

8/7/2019  
Program has improved coordinate usage using normalization and denormalization methods. Camera viewpoint is now
calibrated, allowing for usage of essential matrix transformation over fundamental matrix transformation. Also
fixed the bad interpreter issue. Users are now allowed to run the program using *./slam.py* in coexistence with
the usual *python3 slam.py*.

8/2/2019  
Improved image feature extraction of the program using built in opencv methods, making for cleaner, better
performing feature extracting. Also modified and did general code cleanup to improve overall readability
with additional frame information printing.

7/31/2019  
Added initial basic feature extracting features to the program. Image frame is used to slice image into
separate grids and check for features to track.

7/31/2019  
Making use of opencv and sdl2 python libraries. Display class houses the sdl2 frame display method.
Note that the repository will not include a .mp4 video file to work with. Please provide your own video file.
