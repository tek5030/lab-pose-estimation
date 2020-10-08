#include "camera_projection_measurement.h"

CameraProjectionMeasurement::CameraProjectionMeasurement(
    const Eigen::Vector2d& normalized_plane_point,
    const Eigen::Vector3d& world_point)
    : world_point_{world_point}
    , normalized_plane_point_{normalized_plane_point}
{

}

LinearizedCameraProjectionMeasurement CameraProjectionMeasurement::linearize(const Sophus::SE3d& current_state) const
{
  // TODO 7.1: Use current_state (T_w_c) and world_point_ (x_w) to predict x_c.
  // Transform world point to camera coordinate frame based on current state estimate.
  Eigen::Vector3d x_c_pred;

  // TODO 7.2: Use x_c_pred to predict x_n.
  // Predict normalized image coordinate based on current state estimate.
  Eigen::Vector2d x_n_pred;

  // Construct linearization object.
  LinearizedCameraProjectionMeasurement linearization;

  // TODO 7.3: Use normalized_plane_point_ to compute the measurement error.
  // Compute measurement error.
  linearization.b; // = ?

  // TODO 7.4: Use the predicted x_c_pred and x_n_pred to compute the measurement Jacobian.
  // Compute measurement Jacobian.
  linearization.A; // = ?

  return linearization;
}
