package frc.quixlib.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;

public class QuixSwerveModuleFactory {
  private final double m_wheelCircumference;
  private final MechanismRatio m_driveRatio;
  private final MechanismRatio m_steeringRatio;
  private final PIDConfig m_drivePIDConfig;
  private final SimpleMotorFeedforward m_driveFeedforward;
  private final PIDConfig m_steeringPIDConfig;

  public QuixSwerveModuleFactory(
      double wheelCircumference,
      MechanismRatio driveRatio,
      MechanismRatio steeringRatio,
      PIDConfig drivePIDConfig,
      SimpleMotorFeedforward driveFeedforward,
      PIDConfig steeringPIDConfig) {
    m_wheelCircumference = wheelCircumference;
    m_steeringRatio = steeringRatio;
    m_driveRatio = driveRatio;
    m_drivePIDConfig = drivePIDConfig;
    m_driveFeedforward = driveFeedforward;
    m_steeringPIDConfig = steeringPIDConfig;
  }

  public QuixSwerveModule createModule(
      Translation2d position,
      int driveMotorID,
      int steeringMotorID,
      int absEncoderID,
      double absEncoderOffsetRad) {
    return new QuixSwerveModule(
        position,
        driveMotorID,
        steeringMotorID,
        absEncoderID,
        m_drivePIDConfig,
        m_driveFeedforward,
        m_steeringPIDConfig,
        m_driveRatio,
        m_steeringRatio,
        absEncoderOffsetRad,
        m_wheelCircumference);
  }
}
