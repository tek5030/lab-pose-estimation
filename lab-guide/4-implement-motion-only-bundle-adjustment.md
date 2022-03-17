# Step 4: Implement motion-only bundle adjustment
We will now finish the class `CameraProjectionMeasurement` used by `MobaPoseEstimator`.
This will let us refine the estimated pose of the camera, by minimizing the reprojection error using non-linear optimization.

First, take a look at the definition and documentation in [moba_pose_estimator.h](../moba_pose_estimator.h). 
Then go to [moba_pose_estimator.cpp](../moba_pose_estimator.cpp). 
Read through the code to get an overview. 
Study `MobaPoseEstimator::estimate()`, and `MobaPoseEstimator::optimize()`, and try to understand what is happening here.

Then take a look at [camera_projection_measurement.h](../camera_projection_measurement.h) and go to [camera_projection_measurement.cpp](../camera_projection_measurement.cpp). 
You will now finish the `MobaPoseEstimator` by implementing the linearization of the measurement prediction function in `CameraProjectionMeasurement`.

## 7. Linearize the measurement prediction function
Follow the steps in `CameraProjectionMeasurement::linearize()` to compute **A**<sub>i</sub> and **b**<sub>i</sub>.

Then change the pose estimator used in `runPoseEstimationLab()` in [lab_pose_estimation.cpp](../lab_pose_estimation.cpp) to `MoboPoseEstimator`. 
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
- Add more 3D objects to the AR-world.
- Calibrate your camera!
