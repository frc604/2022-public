package com._604robotics.robotnik.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import java.util.List;

public class TrajectoryCreator {
  private DifferentialDriveKinematics kinematics;
  private TrajectoryConstraint[] constraints;

  private double maxSpeed;
  private double maxAcceleration;

  public TrajectoryCreator(
      DifferentialDriveKinematics kinematics,
      TrackerConstants constants,
      TrajectoryConstraint... constraints) {
    this.constraints = constraints;
    this.kinematics = kinematics;
    maxSpeed = constants.maxSpeed;
    maxAcceleration = constants.maxAcceleration;
  }

  public Trajectory getTrajectory(List<Pose2d> waypoints, boolean reverse) {
    var config = new TrajectoryConfig(maxSpeed, maxAcceleration);
    config.setKinematics(kinematics);

    for (TrajectoryConstraint i : constraints) {
      config.addConstraint(i);
    }

    config.setReversed(reverse);

    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }
}
