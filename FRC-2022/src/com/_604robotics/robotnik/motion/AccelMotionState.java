package com._604robotics.robotnik.motion;

public class AccelMotionState {
  public double position;
  public double velocity;
  public double acceleration;

  public AccelMotionState(double position, double velocity, double acceleration) {
    this.position = position;
    this.velocity = velocity;
    this.acceleration = acceleration;
  }

  public AccelMotionState() {
    this(0.0, 0.0, 0.0);
  }
}
