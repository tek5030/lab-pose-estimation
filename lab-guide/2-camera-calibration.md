# Step 2: Camera calibration
To estimate the pose of the camera using map-image correspondences, we will first need to specify the camera calibration parameters.

## 1. Specify the camera calibration
Fill the correct **K** matrix and distortion parameters into `setupCameraModel()` in [lab_6.cpp:40](https://github.com/tek5030/lab_06/blob/master/lab_6.cpp#L40).

- If you are using the Microsoft webcam, you can use the calibration results in [cameraParameters.xml](https://github.com/tek5030/lab_06/blob/master/cameraParameters.xml).
- If you are using your own camera, [you need to calibrate it](https://docs.opencv.org/4.0.1/d7/d21/tutorial_interactive_calibration.html).

Then, please continue to the [next step](3-implement-homography-based-pose-estimation.md).