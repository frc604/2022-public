package frc.quixlib.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class QuixTalonFX implements QuixMotorControllerWithEncoder, AutoCloseable {
  private static final int kCANTimeoutMs = 100; // ms
  private final WPI_TalonFX m_controller;
  private final TalonFXSimCollection m_simCollection;
  private final MechanismRatio m_ratio;
  private final QuixTalonFXConfiguration m_config;

  public static class QuixTalonFXConfiguration {
    private NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
    private boolean ENABLE_VOLTAGE_COMPENSATION = true;
    private double VOLTAGE_COMPENSATION_SATURATION = 12.0; // V
    private double NEUTRAL_DEADBAND = 0.01; // 1 %
    private boolean INVERTED = false;
    private boolean ENABLE_CURRENT_LIMIT = true;
    private double CURRENT_LIMIT = 40; // A
    private double CURRENT_LIMIT_TRIGGER_THRESHOLD_TIME = 0.1; // s

    public QuixTalonFXConfiguration setBrakeMode() {
      NEUTRAL_MODE = NeutralMode.Brake;
      return this;
    }

    public QuixTalonFXConfiguration setInverted(boolean inverted) {
      INVERTED = inverted;
      return this;
    }

    public QuixTalonFXConfiguration setCurrentLimit(double amps) {
      CURRENT_LIMIT = amps;
      return this;
    }
  }

  public static QuixTalonFXConfiguration makeDefaultConfig() {
    return new QuixTalonFXConfiguration();
  }

  /** Default constructor */
  public QuixTalonFX(int canID, MechanismRatio ratio) {
    this(canID, ratio, makeDefaultConfig());
  }

  public QuixTalonFX(int canID, MechanismRatio ratio, boolean inverted) {
    this(canID, ratio, makeDefaultConfig().setInverted(inverted));
  }

  /** Follower constructor */
  public QuixTalonFX(int canID, QuixTalonFX leader) {
    this(canID, new MechanismRatio()); // Mechanism ratio for a follower is not used.
    m_controller.set(ControlMode.Follower, leader.getDeviceID());
  }

  /** Constructor with full configuration */
  public QuixTalonFX(int canID, MechanismRatio ratio, QuixTalonFXConfiguration config) {
    m_controller = new WPI_TalonFX(canID);
    m_simCollection = m_controller.getSimCollection();
    m_ratio = ratio;
    m_config = config;

    // Set motor controller configuration.
    reportError(m_controller.configFactoryDefault(kCANTimeoutMs), "configFactoryDefault");
    m_controller.setNeutralMode(config.NEUTRAL_MODE);
    m_controller.enableVoltageCompensation(config.ENABLE_VOLTAGE_COMPENSATION);
    reportError(
        m_controller.configVoltageCompSaturation(
            config.VOLTAGE_COMPENSATION_SATURATION, kCANTimeoutMs),
        "configVoltageCompSaturation");
    reportError(
        m_controller.configNeutralDeadband(config.NEUTRAL_DEADBAND, kCANTimeoutMs),
        "configNeutralDeadband");
    m_controller.setInverted(config.INVERTED);
    reportError(
        m_controller.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(
                config.ENABLE_CURRENT_LIMIT,
                config.CURRENT_LIMIT,
                config.CURRENT_LIMIT,
                config.CURRENT_LIMIT_TRIGGER_THRESHOLD_TIME),
            kCANTimeoutMs),
        "configSupplyCurrentLimit");
    reportError(
        m_controller.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(
                config.ENABLE_CURRENT_LIMIT,
                config.CURRENT_LIMIT,
                config.CURRENT_LIMIT,
                config.CURRENT_LIMIT_TRIGGER_THRESHOLD_TIME),
            kCANTimeoutMs),
        "configStatorCurrentLimit");
    reportError(
        m_controller.configIntegratedSensorInitializationStrategy(
            SensorInitializationStrategy.BootToZero, kCANTimeoutMs),
        "configIntegratedSensorInitializationStrategy");

    // Reduce rate of unnecessary CAN frames.
    // https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html
    reportError(
        m_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kCANTimeoutMs),
        "Status_1_General");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_2_Feedback0, 20, kCANTimeoutMs),
        "Status_2_Feedback0");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_3_Quadrature, 1000, kCANTimeoutMs),
        "Status_3_Quadrature");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_4_AinTempVbat, 1000, kCANTimeoutMs),
        "Status_4_AinTempVbat");
    reportError(
        m_controller.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 1000, kCANTimeoutMs),
        "Status_6_Misc");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_7_CommStatus, 1000, kCANTimeoutMs),
        "Status_7_CommStatus");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_8_PulseWidth, 1000, kCANTimeoutMs),
        "Status_8_PulseWidth");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_9_MotProfBuffer, 1000, kCANTimeoutMs),
        "Status_9_MotProfBuffer");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_10_Targets, 1000, kCANTimeoutMs),
        "Status_10_Targets");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_12_Feedback1, 1000, kCANTimeoutMs),
        "Status_12_Feedback1");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kCANTimeoutMs),
        "Status_13_Base_PIDF0");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000, kCANTimeoutMs),
        "Status_14_Turn_PIDF1");
    reportError(
        m_controller.setStatusFramePeriod(
            StatusFrameEnhanced.Status_15_FirmwareApiStatus, 1000, kCANTimeoutMs),
        "Status_15_FirmwareApiStatus");
  }

  public void close() {
    m_controller.close();
  }

  public int getDeviceID() {
    return m_controller.getDeviceID();
  }

  public void setPercentOutput(double percent) {
    m_controller.set(ControlMode.PercentOutput, percent);
  }

  public void setVoltageOutput(double voltage) {
    m_controller.set(ControlMode.PercentOutput, voltage / getMaxVoltage());
  }

  public void setPIDConfig(int slot, PIDConfig config) {
    // TODO: convert gains to falcon units for ease of tuning.
    reportError(m_controller.config_kP(slot, config.getP(), kCANTimeoutMs), "config_kP");
    reportError(m_controller.config_kI(slot, config.getI(), kCANTimeoutMs), "config_kI");
    reportError(m_controller.config_kD(slot, config.getD(), kCANTimeoutMs), "config_kD");
    reportError(m_controller.config_kF(slot, config.getF(), kCANTimeoutMs), "config_kF");
  }

  public void setPositionSetpoint(int slot, double setpoint) {
    m_controller.selectProfileSlot(slot, 0);
    setPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setPositionSetpoint(int slot, double setpoint, double feedforwardVolts) {
    m_controller.selectProfileSlot(slot, 0);
    m_controller.set(
        ControlMode.Position,
        toNativeSensorPosition(setpoint),
        DemandType.ArbitraryFeedForward,
        feedforwardVolts / getMaxVoltage());
  }

  public void setVelocitySetpoint(int slot, double setpoint) {
    m_controller.selectProfileSlot(slot, 0);
    setVelocitySetpoint(slot, setpoint, 0.0);
  }

  public void setVelocitySetpoint(int slot, double setpoint, double feedforwardVolts) {
    m_controller.selectProfileSlot(slot, 0);
    m_controller.set(
        ControlMode.Velocity,
        toNativeSensorVelocity(setpoint),
        DemandType.ArbitraryFeedForward,
        feedforwardVolts / getMaxVoltage());
  }

  public double getMaxVoltage() {
    return m_config.ENABLE_VOLTAGE_COMPENSATION
        ? m_config.VOLTAGE_COMPENSATION_SATURATION
        : RobotController.getBatteryVoltage();
  }

  public double getPercentOutput() {
    return m_controller.getMotorOutputPercent();
  }

  public double getPhysicalPercentOutput() {
    return (getInverted() ? -1.0 : 1.0) * m_controller.getMotorOutputPercent();
  }

  public double getVoltageOutput() {
    return m_controller.getMotorOutputVoltage();
  }

  public boolean getInverted() {
    return m_controller.getInverted();
  }

  public void zeroSensorPosition() {
    setSensorPosition(0.0);
  }

  public void setSensorPosition(double pos) {
    m_controller.setSelectedSensorPosition(toNativeSensorPosition(pos));
  }

  public double getSensorPosition() {
    return fromNativeSensorPosition(m_controller.getSelectedSensorPosition());
  }

  public double getSensorVelocity() {
    return fromNativeSensorVelocity(m_controller.getSelectedSensorVelocity());
  }

  public MechanismRatio getMechanismRatio() {
    return m_ratio;
  }

  public double toNativeSensorPosition(double pos) {
    return toNativeSensorPosition(pos, m_ratio);
  }

  public double toNativeSensorPosition(double pos, MechanismRatio mr) {
    final double motorRadians = mr.mechanismPositionToSensorRadians(pos);
    // Native position is ticks. There are 2048 ticks per revolution.
    return motorRadians * 2048.0 / (2.0 * Math.PI);
  }

  public double fromNativeSensorPosition(double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  public double toNativeSensorVelocity(double vel) {
    return toNativeSensorVelocity(vel, m_ratio);
  }

  public double toNativeSensorVelocity(double vel, MechanismRatio mr) {
    // Native velocity is ticks per 0.1s.
    return toNativeSensorPosition(vel, mr) * 0.1;
  }

  public double fromNativeSensorVelocity(double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorPositionAndVelocity(
      double pos, double vel, double dt, MechanismRatio mr) {
    // Convert position into Falcon ticks.
    final double ticks = toNativeSensorPosition(pos, mr);
    // Convert velocity into Falcon ticks per 100ms.
    final double ticksPer100ms = toNativeSensorVelocity(vel, mr);
    // Simulated hardware is never inverted, so flip signs accordingly.
    final double sign = m_controller.getInverted() ? -1.0 : 1.0;
    m_simCollection.setIntegratedSensorVelocity((int) (sign * ticksPer100ms));
    m_simCollection.setIntegratedSensorRawPosition((int) (sign * ticks));
  }

  public void setSimSensorVelocity(double vel, double dt, MechanismRatio mr) {
    // Convert velocity into Falcon ticks per 100ms.
    final double ticksPer100ms = toNativeSensorVelocity(vel, mr);
    // Simulated hardware is never inverted, so flip signs accordingly.
    final double sign = m_controller.getInverted() ? -1.0 : 1.0;
    m_simCollection.setIntegratedSensorVelocity((int) (sign * ticksPer100ms));
    m_simCollection.addIntegratedSensorPosition((int) (sign * ticksPer100ms * 10.0 * dt));
  }

  private void reportError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(message + ": " + errorCode, /*printTrace=*/ false);
    }
  }
}
