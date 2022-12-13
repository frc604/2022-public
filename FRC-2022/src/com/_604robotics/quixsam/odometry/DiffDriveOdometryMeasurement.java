package com._604robotics.quixsam.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

public class DiffDriveOdometryMeasurement {
  private final Rotation2d gyroAngle;
  private final double leftDistanceMeters;
  private final double rightDistanceMeters;

  private final double sigmaX;
  private final double sigmaY;
  private final Rotation2d sigmaTheta;

  public DiffDriveOdometryMeasurement(
      Rotation2d gyroAngle,
      double leftDistanceMeters,
      double rightDistanceMeters,
      double sigmaX,
      double sigmaY,
      Rotation2d sigmaTheta) {
    this.gyroAngle = gyroAngle;
    this.leftDistanceMeters = leftDistanceMeters;
    this.rightDistanceMeters = rightDistanceMeters;

    this.sigmaX = sigmaX;
    this.sigmaY = sigmaY;
    this.sigmaTheta = sigmaTheta;
  }

  public Rotation2d getGyroAngle() {
    return gyroAngle;
  }

  public double getLeftDistanceMeters() {
    return leftDistanceMeters;
  }

  public double getRightDistanceMeters() {
    return rightDistanceMeters;
  }

  public double getSigmaX() {
    return sigmaX;
  }

  public double getSigmaY() {
    return sigmaY;
  }

  public Rotation2d getSigmaTheta() {
    return sigmaTheta;
  }

  public void addTo(DifferentialDriveOdometry odometry) {
    odometry.update(gyroAngle, leftDistanceMeters, rightDistanceMeters);
  }
}
