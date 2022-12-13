package frc.quixlib.localization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Contains everything necessary to update swerve drive odometry. */
public class SwerveDriveOdometryMeasurement {
  private final Rotation2d m_gyroAngle;
  private final SwerveModulePosition[] m_modulePositions;

  public SwerveDriveOdometryMeasurement(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    m_gyroAngle = gyroAngle;
    m_modulePositions = modulePositions;
  }

  public Rotation2d getGyroAngle() {
    return m_gyroAngle;
  }

  public SwerveModulePosition[] getModulePositionStates() {
    return m_modulePositions;
  }
}
