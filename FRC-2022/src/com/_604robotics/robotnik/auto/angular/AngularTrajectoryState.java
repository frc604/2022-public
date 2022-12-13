package com._604robotics.robotnik.auto.angular;

import edu.wpi.first.math.geometry.Pose2d;

public class AngularTrajectoryState {
  public double time;
  public Pose2d pose;
  public double angularVelocity;

  public AngularTrajectoryState(double time, Pose2d pose, double angularVelocity) {
    this.time = time;
    this.pose = pose;
    this.angularVelocity = angularVelocity;
  }
}
