package com._604robotics.robotnik.motion;

public class MotionConstraints {
  public double maxVelocity;
  public double maxAcceleration;
  public double maxJerk = Double.NaN;

  public MotionConstraints(double maxVelocity, double maxAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }

  public MotionConstraints(double maxVelocity, double maxAcceleration, double maxJerk) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxJerk = maxJerk;
  }
}
