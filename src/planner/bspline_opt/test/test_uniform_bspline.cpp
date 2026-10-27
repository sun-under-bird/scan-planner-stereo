#include <gtest/gtest.h>

#include <Eigen/Core>
#include <bspline_opt/uniform_bspline.h>

TEST(UniformBspline, EvaluatesLinearControlPoints)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  scan_planner::UniformBspline spline(points, 3, 1.0);
  EXPECT_NEAR(spline.evaluateDeBoorT(0.0).x(), 1.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).x(), 3.0, 1e-9);
  EXPECT_NEAR(spline.evaluateDeBoorT(2.0).y(), 0.0, 1e-9);
}

TEST(UniformBspline, DerivativeMatchesLinearSlope)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < points.cols(); ++i) points(0, i) = static_cast<double>(i);

  auto derivative = scan_planner::UniformBspline(points, 3, 0.5).getDerivative();
  const Eigen::Vector3d velocity = derivative.evaluateDeBoorT(0.75);
  EXPECT_NEAR(velocity.x(), 2.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.z(), 0.0, 1e-9);
}

TEST(UniformBspline, TimeScalingPreservesGeometryAndScalesDerivativesOnShortTrajectories)
{
  for (int count : {6, 9, 18})
  {
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, count);
    for (int i = 0; i < count; ++i) {
      points(0, i) = 0.1 * i * i;
      points(1, i) = 0.03 * i;
    }
    scan_planner::UniformBspline before(points, 3, 0.2), after = before;
    after.lengthenTime(2.0);
    ASSERT_NEAR(after.getTimeSum(), 2.0 * before.getTimeSum(), 1e-9);
    auto vb = before.getDerivative(), va = after.getDerivative();
    auto ab = vb.getDerivative(), aa = va.getDerivative();
    for (int k = 0; k <= 20; ++k) {
      const double t = k * before.getTimeSum() / 20.0;
      EXPECT_LT((before.evaluateDeBoorT(t) - after.evaluateDeBoorT(2.0*t)).norm(), 1e-8);
      EXPECT_LT((vb.evaluateDeBoorT(t)/2.0 - va.evaluateDeBoorT(2.0*t)).norm(), 1e-8);
      EXPECT_LT((ab.evaluateDeBoorT(t)/4.0 - aa.evaluateDeBoorT(2.0*t)).norm(), 1e-8);
    }
  }
}

TEST(UniformBspline, DiagonalLimitsUseVectorMagnitude)
{
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(3, 6);
  for (int i = 0; i < 6; ++i) points.col(i) = Eigen::Vector3d(0.6*i, 0.6*i, 0.0);
  scan_planner::UniformBspline spline(points, 3, 1.0);
  spline.setPhysicalLimits(0.75, 0.5, 0.0);
  double ratio = 0;
  EXPECT_FALSE(spline.checkFeasibility(ratio));
  EXPECT_NEAR(ratio, std::sqrt(2.0)*0.6/0.75, 1e-9);
}
