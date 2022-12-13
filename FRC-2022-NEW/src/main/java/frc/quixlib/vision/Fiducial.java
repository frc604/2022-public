package frc.quixlib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Fiducial {
  private final Type m_type;
  // An ID of -1 indicates this is an unlabeled fiducial (e.g. retroreflective tape)
  private final int m_id;
  private final Pose3d m_pose;

  private static final double kXOffset = Units.feetToMeters(54 * 0.5);
  private static final double kYOffset = Units.feetToMeters(27 * 0.5);

  public static enum Type {
    RETROREFLECTIVE,
    APRILTAG,
  }

  public Fiducial(double x, double y, double z) {
    this(Type.RETROREFLECTIVE, -1, new Pose3d(x, y, z, new Rotation3d()));
  }

  public Fiducial(Type type, int id, Pose3d pose) {
    m_type = type;
    m_id = id;
    m_pose = pose;
  }

  public Type getType() {
    return m_type;
  }

  public int id() {
    return m_id;
  }

  public Pose3d getPose() {
    return m_pose;
  }

  public double getX() {
    return m_pose.getX();
  }

  public double getY() {
    return m_pose.getY();
  }

  public double getZ() {
    return m_pose.getZ();
  }

  public double getXRot() {
    return m_pose.getRotation().getX();
  }

  public double getYRot() {
    return m_pose.getRotation().getY();
  }

  public double getZRot() {
    return m_pose.getRotation().getZ();
  }

  public static Fiducial aprilTagFromBlueAllianceCoords(
      int id, double x, double y, double z, double xRot, double yRot, double zRot) {
    return new Fiducial(
        Type.APRILTAG,
        id,
        new Pose3d(x - kXOffset, y - kYOffset, z, new Rotation3d(xRot, yRot, zRot)));
  }
}
