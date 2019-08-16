This is a simple, evolving simultaneous localization and mapping program.
This project is currently being developed using Python 3.x on Ubuntu 18.04 operating system. The program is currently in the feature and 
infromation extraction stage with map construction as a future goal of the project.

**How to run the program**  
After cloning this repository, run the program using *./slam.py* or *python3 slam.py*

**Current Updates:**  

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
