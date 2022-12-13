package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class NTPoseEstimate {
  private int id;
  private Pose2d pose;

  public NTPoseEstimate(int id, Pose2d pose) {
    this.id = id;
    this.pose = pose;
  }

  public int getID() {
    return id;
  }

  public Pose2d getPose() {
    return pose;
  }
}
