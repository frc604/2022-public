package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class NTOdometryMeasurement {
  private int m_id;
  private Pose2d m_pose;
  private Pose2d m_sigmas;

  public NTOdometryMeasurement(int id, Pose2d pose, Pose2d sigmas) {
    m_id = id;
    m_pose = pose;
    m_sigmas = sigmas;
  }

  public void setId(int id) {
    m_id = id;
  }

  public double[] toArray() {
    return new double[] {
      (double) m_id,
      m_pose.getX(),
      m_pose.getY(),
      m_pose.getRotation().getRadians(),
      m_sigmas.getX(),
      m_sigmas.getY(),
      m_sigmas.getRotation().getRadians()
    };
  }
}
