# Particle Filter-Based SLAM for Humanoid Robot

Simultaneous Localization and Mapping (SLAM) problem is a well-known problem in robotics, where a robot has to localize itself and map its environment simultaneously. Particle filter (PF) is one of the most adapted estimation algorithms for SLAM apart from Kalman filter (KF) and Extended Kalman Filter (EKF).\
Here I implement SLAM using a particle filter on data collected from a humanoid named THOR that was built at Penn and UCLA.

### Hardware Setup of THOR

The humanoid THOR has a [Hokuyo LiDAR sensor](https://hokuyo-usa.com/products/lidar-obstacle-detection) on its head. This LiDAR is a planar LiDAR sensor and returns 1080 readings at each instant, each reading being the distance of some physical object along a ray that shoots off at an angle between (-135, 135) degrees with discretization of 0.25 degrees in an horizontal plane.

![image](https://user-images.githubusercontent.com/38180831/205529594-cda1e25e-a384-4bf2-b0ca-0a23c0719735.png)

Here I use the position and orientation of the head of the robot to calculate the orientation of the LiDAR in the body frame. The second kind of observations I used pertain to the location of the robot. I  directly used the (x,y,θ) pose of the robot in the world coordinates (θ denotes yaw). These poses were created presumably on the robot by running a filter on the IMU data (such estimates are called odometry estimates), and these poses will not be extremely accurate.

You can read more about the hardware in this paper - [THOR-OP humanoid robot for DARPA Robotics Challenge Trials 2013](https://ieeexplore.ieee.org/document/7057369)

### Coordinate frames
The body frame is at the top of the head (X axis pointing forwards, Y axis pointing left and Z axis pointing upwards), the top of the head is at a height of 1.263m from the ground. The transformation from the body frame to the LiDAR frame depends upon the angle of the head (pitch) and the angle of the neck (yaw) and the height of the LiDAR above the head (which is 0.15m). The world coordinate frame where we want to build the map has its origin on the ground plane, i.e., the origin of the body frame is at a height of 1.263m with respect to the world frame at location (x,y,θ).

## Running the code
1. You can choose to run the different parts of the SLAM algorithm (dynamic step and observation step) either separately or together. To do this, pass a mode argument, either 'dynamics', 'observation', or 'slam', in the main function of `main.py`.

2. Run the `main.py` file and set the datasets you want to use by passing the idx argument corresponding to the desired dataset.

## Results
#### The odometry and dynamics plots for dynamics step:
![image](https://user-images.githubusercontent.com/38180831/205530920-da7610a4-c5e7-4945-b67f-e5a62ad188a8.png)

#### SLAM on Dataset 0, Q = 1e-4
![image](https://user-images.githubusercontent.com/38180831/205531075-afd27c93-64e5-4308-9c82-eb8314a3e8e6.png)

#### SLAM on Dataset 1, Q = 1e-4
![image](https://user-images.githubusercontent.com/38180831/205531091-a416a0a9-8ca0-43b2-9d24-d3ddf831f42f.png)

#### SLAM on Dataset 2, Q = 1e-4
![image](https://user-images.githubusercontent.com/38180831/205531113-20179569-e17b-46f0-95d6-472f5e6cd8a6.png)

#### SLAM on Dataset 3, Q = 1e-4
![image](https://user-images.githubusercontent.com/38180831/205531138-5b250879-e669-4000-808f-d9a22ff5057e.png)

#### SLAM on Dataset 0, Q = 1e-8
![image](https://user-images.githubusercontent.com/38180831/205531273-6b538618-f2b2-4fcf-b106-e601c310ebb8.png)

#### SLAM on Dataset 1, Q = 1e-8
![image](https://user-images.githubusercontent.com/38180831/205531287-10059179-fdb3-400f-af8d-ad19681cfddc.png)

#### SLAM on Dataset 2, Q = 1e-8
![image](https://user-images.githubusercontent.com/38180831/205531302-c2bb3916-bb0f-47e3-968e-7e9e4ea5bb15.png)

#### SLAM on Dataset 3, Q = 1e-8
![image](https://user-images.githubusercontent.com/38180831/205531318-77a01060-96cb-4e35-91e5-4c33f2bdcfab.png)
