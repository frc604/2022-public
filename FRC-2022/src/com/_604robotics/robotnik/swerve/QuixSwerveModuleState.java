// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robotnik.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Represents the state of one swerve module. */
@SuppressWarnings("MemberName")
public class QuixSwerveModuleState implements Comparable<QuixSwerveModuleState> {
  /** Speed of the wheel of the module. */
  public double speedMetersPerSecond;

  /** Angle of the module. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public QuixSwerveModuleState() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public QuixSwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
  }

  /**
   * Compares two swerve module states. One swerve module is "greater" than the other if its speed
   * is higher than the other.
   *
   * @param o The other swerve module.
   * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
   */
  @Override
  @SuppressWarnings("ParameterName")
  public int compareTo(QuixSwerveModuleState o) {
    return Double.compare(this.speedMetersPerSecond, o.speedMetersPerSecond);
  }

  @Override
  public String toString() {
    return String.format(
        "SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speedMetersPerSecond, angle);
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static QuixSwerveModuleState optimize(QuixSwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new QuixSwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360 - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360;
    }
    while (newAngle > upperBound) {
        newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
        newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
        newAngle += 360;
    }
    return newAngle;
  }
}