package com._604robotics.robotnik.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;

public class FalconDashboard {
  private final NetworkTableInstance network = NetworkTableInstance.getDefault();
  private final NetworkTable table = network.getTable("Live_Dashboard");

  private static FalconDashboard single_instance = null;

  public FalconDashboard() {}

  public static FalconDashboard getInstance() {
    if (single_instance == null) single_instance = new FalconDashboard();

    return single_instance;
  }

  public void publishRobotPose(Pose2d pose) {
    table.getEntry("robotX").setDouble(pose.getTranslation().getX() * 3.281 + 27);
    table.getEntry("robotY").setDouble(pose.getTranslation().getY() * 3.281 + 13.5);
    table.getEntry("robotHeading").setDouble(pose.getRotation().getRadians());
  }

  public void publishPathPose(Pose2d pose) {
    table.getEntry("pathX").setDouble(pose.getTranslation().getX() * 3.281 + 27);
    table.getEntry("pathY").setDouble(pose.getTranslation().getY() * 3.281 + 13.5);
    table.getEntry("pathHeading").setDouble(pose.getRotation().getRadians());
    table.getEntry("isFollowingPath").setBoolean(true);
  }
}
