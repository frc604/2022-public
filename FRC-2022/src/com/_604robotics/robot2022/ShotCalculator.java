package com._604robotics.robot2022;

import com._604robotics.robot2022.subsystems.Swerve;
import com._604robotics.robotnik.math.MathUtils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class ShotCalculator {
  private Swerve swerve;

  public ShotCalculator(Swerve swerve) {
    this.swerve = swerve;
  }

  // The desired robot yaw and ball exit velocity given the current robot position and velocity.
  // Also includes a boolean to indicate whether the shot is currently feasible. Shots may be
  // infeasible if the robot is too far or moving too quickly.
  public class ShotInfo {
    public final double yaw;
    public final double velocity;
    public final boolean feasible;

    public ShotInfo(double yaw, double velocity, boolean feasible) {
      this.yaw = yaw;
      this.velocity = velocity;
      this.feasible = feasible;
    }
  }

  class YawInfo {
    private final double yaw;
    private final boolean feasible;

    public YawInfo(double yaw, boolean feasible) {
      this.yaw = yaw;
      this.feasible = feasible;
    }
  }

  private YawInfo computeTargetYaw(
      Pose2d pose,
      Matrix<N2, N1> goalVec,
      Matrix<N2, N1> vRobotFieldVelVec,
      double ballExitVel) {
    // Returns the yaw such that the ball horizontal vector is aligned with the goal vector.
    double ballHorizVel = ballExitVel * Math.cos(Calibration.Launcher.launchAngle);
    // We require ballHorizVel to be some margin greater than the robot's goal-aligned velocity,
    // or else the ball will end up going backwards, which results in undefined behavior.
    // If it is not, we set the velocity to meet this margin, and mark the feasibility as false.
    boolean feasible = true;
    double goalAlignedVel = vRobotFieldVelVec.transpose().times(goalVec).get(0, 0) / goalVec.normF();
    double velWithMargin = goalAlignedVel + 1.0;
    if (ballHorizVel <= velWithMargin) {
      feasible = false;
      ballHorizVel = velWithMargin;
    }

    Matrix<N2, N1> vGoalVec = goalVec.div(goalVec.normF()).times(ballHorizVel);

    // Compute desired vBallVec so that the resulting ball horizonal velocity is vGoalVec, accounting for robot velocity.
    Matrix<N2, N1> vBallVec = vGoalVec.minus(vRobotFieldVelVec);
    double targetYaw = Math.atan2(vBallVec.get(1, 0), vBallVec.get(0, 0));

    if (Double.isNaN(targetYaw)) {
      return new YawInfo(0.0, false);
    }
    return new YawInfo(targetYaw, feasible);
  }

  private boolean isBallExitVelHigh(Pose2d pose, double targetYaw, double ballExitVel) {
    // Returns whether the ballExitVel at this targetYaw is too high.
    // This is used to binary search our way to an appropriate exit velocity.

    // Get goal point in frame. Use 3D frames because we don't have 2D helper utils.
    Matrix<N4, N4> robotFrame = MathUtils.makeZRotTFMatrix(pose.getX(), pose.getY(), 0, targetYaw);
    Matrix<N3, N1> goalPoint = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 0);
    Matrix<N3, N1> goalPointInRobotFrame = MathUtils.getPointInFrame(goalPoint, robotFrame);

    // Compute whether the shot reaches the required distance and height.
    double requiredDistance = goalPointInRobotFrame.get(0, 0);
    double requiredHeight = Calibration.PoseEstimator.goalHeight - Calibration.Launcher.launchHeight;
    double ballVertVel = ballExitVel * Math.sin(Calibration.Launcher.launchAngle);
    double ballHorizVel = ballExitVel * Math.cos(Calibration.Launcher.launchAngle);
    // Use parabolic equations of motion.
    // We only care about the greater solution of |t| for the "down" part of the parabola.
    // requiredHeight - ballVertVel * t + 0.5 * g * t^2 = 0
    double t = (ballVertVel + Math.sqrt(ballVertVel * ballVertVel - 2 * Calibration.g * requiredHeight)) / Calibration.g;
    Matrix<N2, N1> vRobotVec = swerve.getVelocityVector();
    double dist = t * (ballHorizVel + vRobotVec.get(0, 0));
    return dist > requiredDistance;
  }

  public ShotInfo computeShotInfo() {
    // Compute velocity-compensated scoring.
    // Note that this assumes the launcher is centered on the robot, which isn't exactly correct.

    // Vector that points towards the goal, in field coordinates.
    Pose2d pose = swerve.getEstimatedPose();
    double estimatedYaw = pose.getRotation().getRadians();
    double goalX = -pose.getX();
    double goalY = -pose.getY();
    Matrix<N2, N1> goalVec = Matrix.mat(Nat.N2(), Nat.N1()).fill(goalX, goalY);

    // The velocity vector of the robot, in field coordinates.
    Matrix<N2, N1> vRobotFieldVelVec = getFieldRelativeOrbitVelocityVector();

    // Closed form solution is messy. Let's just binary search over |ballExitVel| for the best solution.
    double lbVel = Calibration.Launcher.minVel - 1.0;  // Lower bound
    double ubVel = Calibration.Launcher.maxVel + 1.0;  // Upper bound
    double midVel = 0.0;
    YawInfo yawInfo = new YawInfo(0.0, false);
    for (int i = 0; i < 10; ++i) {   // 10 iterations should give us more than enough precision.
      midVel = (lbVel + ubVel) * 0.5;
      yawInfo = computeTargetYaw(pose, goalVec, vRobotFieldVelVec, midVel);
      boolean isHigh = isBallExitVelHigh(pose, yawInfo.yaw, midVel);
      if (isHigh) {
        ubVel = midVel;
      } else {
        lbVel = midVel;
      }
    }

    boolean feasible = true;
    if (!yawInfo.feasible || midVel <= Calibration.Launcher.minVel || midVel >= Calibration.Launcher.maxVel) {
      feasible = false;
    }
    double unwrappedYaw = MathUtils.unwrapAngle(yawInfo.yaw, estimatedYaw);
    return new ShotInfo(unwrappedYaw, midVel, feasible);
  }

  public Matrix<N2, N1> getFieldRelativeOrbitVelocityVector() {
    return Matrix.mat(Nat.N2(), Nat.N1()).fill(
      swerve.getOrbitTangentialVelocity() * -swerve.getAngleAroundGoal().getSin(),
      swerve.getOrbitTangentialVelocity() * swerve.getAngleAroundGoal().getCos()
    );
  }
}
