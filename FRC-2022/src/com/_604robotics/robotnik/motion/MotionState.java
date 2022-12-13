package com._604robotics.robotnik.motion;

public class MotionState {
  public double position;
  public double velocity;

  public MotionState(double position, double velocity) {
    this.position = position;
    this.velocity = velocity;
  }

  public MotionState() {
    this(0.0, 0.0);
  }
}
