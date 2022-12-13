package com._604robotics.robotnik.auto.angular;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class TurnInPlaceTrajectory {
  private TrapezoidProfile profile;

  public TrapezoidProfile.Constraints constraints;

  private Pose2d initial;
  private Rotation2d end;

  private TrapezoidProfile.State prevRef;

  public TurnInPlaceTrajectory(
      Pose2d initialPose, Rotation2d endRotation, AngularMotionConstraint constraint) {
    initial = initialPose;
    end = endRotation;

    constraints =
        new TrapezoidProfile.Constraints(
            constraint.maxAngularVelocity, constraint.maxAngularAcceleration);

    profile =
        new TrapezoidProfile(
            constraints,
            new State(end.getRadians(), 0),
            new State(initial.getRotation().getRadians(), 0));

    prevRef = new TrapezoidProfile.State(initial.getRotation().getRadians(), 0.0);
  }

  public Pose2d getInitialPose() {
    return initial;
  }

  public AngularTrajectoryState sample(double seconds, double dt) {
    profile = new TrapezoidProfile(constraints, new State(end.getRadians(), 0), prevRef);

    TrapezoidProfile.State state = profile.calculate(dt);

    SmartDashboard.putNumber("State", new Rotation2d(state.position).getRadians());

    SmartDashboard.putNumber("Time", profile.totalTime());

    prevRef = state;

    return new AngularTrajectoryState(
        seconds,
        new Pose2d(initial.getTranslation(), new Rotation2d(state.position)),
        state.velocity);
  }

  public double getTotalTimeSeconds() {
    return profile.totalTime();
  }
}
