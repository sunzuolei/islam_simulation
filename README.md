# About the project
```
Run the script "run.m", and you can see the demo of EKF-SLAM or IEKF-SLAM by setting the switches. 
Also, you can change the configurations and other switches in "run.m" to get different results.
```
**Note:**

Before run the script "run.m", you should set the path by hand. In the Matlab menu, just click Home->Set path->Add folder, and select the path of the file "IEKF and EKF Feature-based SLAM".

**The following files mean :**
## Data
Data about the robot surroundings and experiments. 
* **rollerCoaster.mat :** waypoints and landmarks.
* **xTrue.mat :** ground truth.
* **idList.mat :** corresponding list.
* **ekfdata.mat :** experimental data of EKF-SLAM.
* **iekfdata.mat :** experimental data of IEKF-SLAM.

## Filter
It contains the filter codes for SLAM execution.
* **main.m :** main loop of feature-based SLAM.
* **predictEKF.m :** predict step of EKF and IEKF SLAM. 
* **updateEKF.m :** update step of EKF-SLAM.
* **updateIEKF.m** and **iterate.m :** update step of IEKF-SLAM.
* **augmentState.m :** augment step, that is add new features to the map.

## Models
It contains all the models of filter.
* **conNoise.m :** add random noise to control vector.
* **vehicleModel.m :** generate ground truth and include motion model.
* **obsModel.m :** observation model.

## Utilities
Tools for SLAM Demo.
* **compound.m :** complete the estimating uncertain spatial relationships in robotics.
* **corresponding.m :** classify features with known corresponding.
* **dataStore.m :** store online data.
* **piTopi.m :** make sure the angle is in [-pi, pi].

## Visualization
Functions for drawing.
* **vis.m :** for animation.
* **getLandmark.m :** get the landmarks within robot's laser range.
* **getSigmaEllipse.m :** For drawing the sigma ellipse which represents the uncertainty.
* **makeFcov.m :** make features' covariance ecllipse.
* **makeLines.m :** make sets of line segments for laser range-bearing measurements.

**The bibtex of this paper is :**</br>
@article{许亚芳2015基于多次测量更新的移动机器人SLAM仿真,</br>
  title={基于多次测量更新的移动机器人SLAM仿真},</br>
  author={许亚芳 and 孙作雷 and 曾连荪 and 张波},</br>
  journal={系统仿真学报},</br>
  year={2015}</br>
}
=======
# islam_simulation
The codes of the paper The Mobile Robot SLAM Simulation with Multi Measurement Update
>>>>>>> e573e89d0983c60592b408df3cf2b0b6f88d91ee
