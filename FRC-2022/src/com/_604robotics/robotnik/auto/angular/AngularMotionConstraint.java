package com._604robotics.robotnik.auto.angular;

public class AngularMotionConstraint {
  public double maxAngularVelocity;
  public double maxAngularAcceleration;

  public AngularMotionConstraint(double maxAngularVelocity, double maxAngularAcceleration) {
    this.maxAngularVelocity = maxAngularVelocity;
    this.maxAngularAcceleration = maxAngularAcceleration;
  }
}
