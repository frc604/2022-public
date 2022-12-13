package frc.quixlib.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.quixlib.devices.QuixAbsoluteEncoder;
import frc.quixlib.devices.QuixCANCoder;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.PIDConfig;
import frc.quixlib.motorcontrol.QuixTalonFX;

public class QuixSwerveModule {
  private final int kDriveVelocityPIDSlot = 0;
  private final int kSteeringPIDSlot = 0;

  private final Translation2d m_position;
  private final double m_absEncoderOffsetRad;
  private final QuixTalonFX m_driveMotor;
  private final QuixTalonFX m_steeringMotor;
  private final SimpleMotorFeedforward m_driveFeedforward;
  private final MechanismRatio m_driveRatio;
  private final MechanismRatio m_steeringRatio;
  private final QuixAbsoluteEncoder m_absSteeringEncoder;
  private final double m_wheelCircumference;

  private SwerveModuleState m_lastCommandedState;
  private double m_steeringZeroPosition = 0.0;

  public QuixSwerveModule(
      Translation2d position,
      int driveMotorID,
      int steeringMotorID,
      int absEncoderID,
      PIDConfig drivePIDConfig,
      SimpleMotorFeedforward driveFeedforward,
      PIDConfig steeringPIDConfig,
      MechanismRatio driveRatio,
      MechanismRatio steeringRatio,
      double absEncoderOffsetRad,
      double wheelCircumference) {
    m_position = position;
    m_absEncoderOffsetRad = absEncoderOffsetRad;
    m_driveMotor =
        new QuixTalonFX(driveMotorID, driveRatio, QuixTalonFX.makeDefaultConfig().setBrakeMode());
    m_driveMotor.setPIDConfig(kDriveVelocityPIDSlot, drivePIDConfig);
    m_driveFeedforward = driveFeedforward;
    m_steeringMotor =
        new QuixTalonFX(
            steeringMotorID, steeringRatio, QuixTalonFX.makeDefaultConfig().setCurrentLimit(30));
    m_steeringMotor.setPIDConfig(kSteeringPIDSlot, steeringPIDConfig);
    m_driveRatio = driveRatio;
    m_steeringRatio = steeringRatio;
    m_absSteeringEncoder = new QuixCANCoder(absEncoderID, new MechanismRatio());
    m_wheelCircumference = wheelCircumference;

    zeroToAbsPosition();
    m_lastCommandedState = getState();

    // For simulation
    double kEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;
    m_driveSim =
        new FlywheelSim(
            DCMotor.getFalcon500(1),
            driveRatio.reduction(),
            0.01,
            VecBuilder.fill(kEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
            );
    m_steeringSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            steeringRatio.reduction(),
            0.001, // MOI
            0.0, // Length (m)
            Double.NEGATIVE_INFINITY, // Min angle
            Double.POSITIVE_INFINITY, // Max angle
            10.0, // Mass (kg)
            false, // Simulate gravity
            VecBuilder.fill(kEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
            );
  }

  public Translation2d getPosition() {
    return m_position;
  }

  public void zeroToAbsPosition() {
    final double absAngle = m_absSteeringEncoder.getAbsPosition() - m_absEncoderOffsetRad;
    m_steeringZeroPosition = m_steeringMotor.getSensorPosition() - absAngle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isClosedLoop) {
    if (isClosedLoop) {
      m_driveMotor.setVelocitySetpoint(
          kDriveVelocityPIDSlot,
          desiredState.speedMetersPerSecond,
          m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    } else {
      m_driveMotor.setVoltageOutput(
          m_driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    m_steeringMotor.setPositionSetpoint(
        kSteeringPIDSlot, desiredState.angle.getRadians() + m_steeringZeroPosition);

    // Save this state
    m_lastCommandedState = desiredState;
  }

  public SwerveModuleState getState() {
    final double velocity = m_driveMotor.getSensorVelocity();
    final double angle = m_steeringMotor.getSensorPosition() - m_steeringZeroPosition;
    return new SwerveModuleState(velocity, new Rotation2d(angle));
  }

  public SwerveModuleState getLastCommandedState() {
    return m_lastCommandedState;
  }

  public SwerveModulePosition getPositionState() {
    final double position = m_driveMotor.getSensorPosition();
    final double angle = m_steeringMotor.getSensorPosition() - m_steeringZeroPosition;
    return new SwerveModulePosition(position, new Rotation2d(angle));
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private final FlywheelSim m_driveSim;
  private final SingleJointedArmSim m_steeringSim;

  /** Simulate one module with naive physics model. */
  public void updateSimPeriodic() {
    // Simulate drive
    m_driveSim.setInput(
        m_driveMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_driveSim.update(TimedRobot.kDefaultPeriod);
    double metersPerSecond =
        m_driveSim.getAngularVelocityRadPerSec() * m_wheelCircumference / (2.0 * Math.PI);
    m_driveMotor.setSimSensorVelocity(metersPerSecond, TimedRobot.kDefaultPeriod, m_driveRatio);

    // Simulate steering
    m_steeringSim.setInput(
        m_steeringMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_steeringSim.update(TimedRobot.kDefaultPeriod);
    m_steeringMotor.setSimSensorPositionAndVelocity(
        m_steeringSim.getAngleRads(),
        m_steeringSim.getVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        m_steeringRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
