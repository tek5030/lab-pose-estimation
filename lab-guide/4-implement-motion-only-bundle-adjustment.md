# Step 4: Implement motion-only bundle adjustment
We will now follow the slides in [lecture 6.4](https://www.uio.no/studier/emner/matnat/its/TEK5030/v20/forelesninger/lecture_6_4_nonlinear-pose-estimation.pdf) to finish the class `CameraProjectionMeasurement`.
This will let us refine the estimated pose of the camera, by minimizing the reprojection error using non-linear optimization.

First, take a look at the definition and documentation in [moba_pose_estimator.h](https://github.com/tek5030/lab_06/blob/master/moba_pose_estimator.h). 
Then go to [moba_pose_estimator.cpp](https://github.com/tek5030/lab_06/blob/master/moba_pose_estimator.cpp). 
Read through the code to get an overview. 
Study `MobaPoseEstimator::estimate()`, and `MobaPoseEstimator::optimize()`, and try to understand what is happening here.

Then take a look at [camera_projection_measurement.h](https://github.com/tek5030/lab_06/blob/master/camera_projection_measurement.h) and go to [camera_projection_measurement.cpp](https://github.com/tek5030/lab_06/blob/master/camera_projection_measurement.cpp). 
You will now finish the `MobaPoseEstimator` by implementing the linearization of the measurement prediction function in `CameraProjectionMeasurement`.

## 7. Linearize the measurement prediction function
Follow the steps in `CameraProjectionMeasurement::linearize()` to compute **A**<sub>i</sub> and **b**<sub>i</sub>.

Then change the pose estimator used in `lab6()` in [lab_6.cpp](https://github.com/tek5030/lab_06/blob/master/lab_6.cpp) to `MoboPoseEstimator`. 
You can use the `HomographyPoseEstimator` as a method to get the initial pose by:

```c++
// Construct pose estimator.
auto init_estimator = std::make_shared<HomographyPoseEstimator>(camera_model.K);
MobaPoseEstimator pose_estimator(init_estimator, camera_model.principalPoint(), camera_model.focalLengths());
```

Compile, run and test!

## Extra
Have even more fun by for example:

- Compare the results with the supplied `PnPPoseEstimator`.
- Change the optimization method from Gauss-Newton to Levenberg-Marquardt.
- Add noise estimates to the measurements by using the scale from the descriptors. 
  Update `MobaPoseEstimator` to take noise into account.
- Add more 3D objects to the AR-world.
