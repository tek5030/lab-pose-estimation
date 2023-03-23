# Pose estimation and Augmented Reality

Welcome to this lab in the computer vision course [TEK5030] at the University of Oslo.

In this lab we will implement pose estimation based on 3D-2D correspondences from scratch!

![Illustration of pose estimation from 3D-2D correspondences](lab-guide/img/pose_estimation.png)

Start by cloning this repository on your machine.
Then open the lab project in your editor.

The lab is carried out by following these steps:
1. [Get an overview](lab-guide/1-get-an-overview.md)
2. [Camera calibration](lab-guide/2-camera-calibration.md)
3. [Implement homography-based pose estimation](lab-guide/3-implement-homography-based-pose-estimation.md)
4. [Implement Motion-only Bundle Adjustment](lab-guide/4-implement-motion-only-bundle-adjustment.md)

You will find our proposed solution at https://github.com/tek5030/solution-pose-estimation.
Please try to solve the lab with help from others instead of just jumping straight to the solution ;)

Please start the lab by going to the [first step](lab-guide/1-get-an-overview.md).

## Prerequisites

For this lab, we can unfortunately not rely on conan to install all required OpenCV modules (namely the `viz` module for 3D visualization). You have a few other options:

- Solve the [python lab](https://github.com/tek5030/lab-pose-estimation-py) (recommended)
- Use the lab computers
- Install OpenCV using [homebrew](https://brew.sh/) (option for mac and linux). (See also [Getting started on MacOS](https://tek5030.github.io/tutorial/macos.html).)
- Rely on virtualbox and our prepared linux image with dependencies preinstalled (see [Canvas: Setting up your computer for the labs](https://uio.instructure.com/courses/44675/discussion_topics/295673))
- Try [Docker toolchain][docker-toolchain] in CLion (very experimental)


[TEK5030]: https://www.uio.no/studier/emner/matnat/its/TEK5030/
[conan]: https://tek5030.github.io/tutorial/conan.html
[lab_intro]: https://github.com/tek5030/lab-intro/blob/master/cpp/lab-guide/1-open-project-in-clion.md#6-configure-project
[docker-toolchain]: https://tek5030.github.io/tutorial/dev-container.html