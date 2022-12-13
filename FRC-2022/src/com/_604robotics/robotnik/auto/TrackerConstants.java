package com._604robotics.robotnik.auto;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class TrackerConstants {
  public SimpleMotorFeedforward feedforward;
  public double kP;
  public double b;
  public double zeta;
  public double maxSpeed;
  public double maxAcceleration;

  public TrackerConstants(
      SimpleMotorFeedforward feedforward,
      double kP,
      double b,
      double zeta,
      double maxSpeed,
      double maxAcceleration) {
    this.feedforward = feedforward;
    this.kP = kP;
    this.b = b;
    this.zeta = zeta;
    this.maxSpeed = maxSpeed;
    this.maxAcceleration = maxAcceleration;
  }
}
