#include "homography_pose_estimator.h"
#include "Eigen/Dense"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"

HomographyPoseEstimator::HomographyPoseEstimator(const Eigen::Matrix3d& K)
    : K_{K}
{ }


PoseEstimate HomographyPoseEstimator::estimate(const std::vector<cv::Point2f>& image_points,
                                               const std::vector<cv::Point3f>& world_points)
{
  // Set a minimum required number of points,
  // here 3 times the theoretic minimum.
  constexpr size_t min_number_points = 12;

  // Check that we have enough points.
  if (image_points.size() < min_number_points)
  {
    return {};
  }

  // Compute the homography and extract the inliers.
  std::vector<char> inliers;
  cv::Mat H_cv = cv::findHomography(world_points, image_points, cv::RANSAC, 3, inliers);

  std::vector<cv::Point2f> inlier_image_points;
  std::vector<cv::Point3f> inlier_world_points;
  for (size_t i=0; i<inliers.size(); ++i)
  {
    if (inliers[i] > 0)
    {
      inlier_image_points.push_back(image_points[i]);
      inlier_world_points.push_back(world_points[i]);
    }
  }

  // Check that we have enough inliers.
  if (inlier_image_points.size() < min_number_points)
  {
    return {};
  }

  // Convert homography to Eigen matrix.
  Eigen::Matrix3d H;
  cv::cv2eigen(H_cv, H);

  // TODO 2: Compute M.
  // Compute the matrix M
  Eigen::Matrix3d M;

  // Extract M_bar (the two first columns of M).
  Eigen::MatrixXd M_bar = M.leftCols<2>();

  // Perform SVD on M_bar.
  auto svd = M_bar.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  // TODO 3: Compute R_bar.
  // Compute R_bar (the two first columns of R)
  // from the result of the SVD.
  Eigen::Matrix<double, 3, 2> R_bar;

  // TODO 4: Construct R.
  // Construct R by inserting R_bar and
  // computing the third column of R from the two first.
  // Remember to check det(R)!
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  // TODO 5: Compute the scale.
  // Compute the scale factor lambda.
  double lambda = 0.0;

  // TODO 6: Find correct solution.
  // Extract the translation t.
  // Check that this is the correct solution
  // by testing the last element of t.
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  // Return camera pose in the world.
  Sophus::SE3d pose_C_W(R, t);
  return {pose_C_W.inverse(), inlier_image_points, inlier_world_points};
}
