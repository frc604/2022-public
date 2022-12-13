package com._604robotics.quixsam.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class SwerveDriveOdometryMeasurement {
  private final Rotation2d gyroAngle;
  private final SwerveModuleState[] moduleStates;

  private final double sigmaX;
  private final double sigmaY;
  private final Rotation2d sigmaTheta;

  public SwerveDriveOdometryMeasurement(
      Rotation2d gyroAngle,
      double sigmaX,
      double sigmaY,
      Rotation2d sigmaTheta,
      SwerveModuleState... moduleStates) {
    this.gyroAngle = gyroAngle;
    this.moduleStates = moduleStates;
    this.sigmaX = sigmaX;
    this.sigmaY = sigmaY;
    this.sigmaTheta = sigmaTheta;
  }

  public Rotation2d getGyroAngle() {
    return gyroAngle;
  }

  public SwerveModuleState[] getModuleStates() {
    return moduleStates;
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
}
