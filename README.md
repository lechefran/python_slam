## What is this project repository about?  
This is a simple, evolving simultaneous localization and mapping program.
This project is currently being developed using Python 3.x on Ubuntu 18.04 operating system. The program is currently in the feature and
information extraction stage with map construction as a future goal of the project.

## How to run the program  
After cloning this repository, run the program using *./slam.py* or *python3 slam.py*

## Required Libraries
This program is being written in Python3 and uses multiple libraries that have to be installed by the user separately.  
Here is a list of the current fundamental Python libaries necessary to the project:  

**OpenCV**  
OpenCV is a library that contains numerous computer vision algorithms. Check them out at [OpenCV](https://docs.opencv.org/3.4/d1/dfb/intro.html).  
To install, use the command `pip3 install opencv-python`.

**PySDL2**  
PySDL2 is a Python wrapper for the SDL2 library. Check them out at [PySDL2](https://pysdl2.readthedocs.io/en/rel_0_9_6/index.html).  
To install, use the command `sudo apt install libsdl2-2.0`.

**Scikit-image**
Scikit-image (skimage) is a collection of image processing algorithms. Check them out at [Scikit-image](https://scikit-image.org/).  
To install, use the command `pip3 install scikit-image`.

## Additional Requirements
Here is a list of additional requirements that are used for this project:

**g2opy**
This is a python binding of graph optimization C++ g2o. Check them out at [g2opy](https://github.com/uoip/g2opy).
You can follow the installation steps using the provided link. Note that there may be problems that occur when following the installation process
provided. In this repository's case, there was a need to modify the common.h file inside pybind11 where include statements such as *#include <Python.h>*
had to be changed to *#include <python3.6/Python.h>* to work. Another change that had to be made was the inclusion of the *PYTHONPATH* of g2opy in
the *~/.bashrc* file of the main directory to include g2opy for the program to work. This can be fixed by modifying the *~/.bashrc* file and adding the
statement *export PYTHONPATH=$PYTHONPATH:path/to/g2opy*.

**Pangolin**
Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input.
Check them out at [Pangolin](https://github.com/stevenlovegrove/Pangolin). Installation instructions are included in the repository link.

It is recommended that you place all these additional repositories in a separate directory to keep you repository clean. Build all of the
additional repositories in that separate directory.

## Current Issues  
Building the additional required libraries may lead to a problem encounter with pybind11. Currently working on a solution that will be posted when resolved.

## Current Updates  
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
