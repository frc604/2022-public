package frc.quixlib.motorcontrol;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class QuixSparkMAX implements QuixMotorControllerWithEncoder, AutoCloseable {
  private final CANSparkMax m_controller;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_pid;
  private final MechanismRatio m_ratio;
  private final QuixSparkMAXConfiguration m_config;

  public static class QuixSparkMAXConfiguration {
    private IdleMode IDLE_MODE = IdleMode.kCoast;
    private boolean ENABLE_VOLTAGE_COMPENSATION = true;
    private double VOLTAGE_COMPENSATION_SATURATION = 12.0; // V
    private boolean INVERTED = false;
    private boolean ENABLE_CURRENT_LIMIT = true;
    private int CURRENT_LIMIT = 40; // A

    public QuixSparkMAXConfiguration setBrakeMode() {
      IDLE_MODE = IdleMode.kBrake;
      return this;
    }

    public QuixSparkMAXConfiguration setInverted(boolean inverted) {
      INVERTED = inverted;
      return this;
    }

    public QuixSparkMAXConfiguration setCurrentLimit(int amps) {
      CURRENT_LIMIT = amps;
      return this;
    }
  }

  public static QuixSparkMAXConfiguration makeDefaultConfig() {
    return new QuixSparkMAXConfiguration();
  }

  /** Default constructor */
  public QuixSparkMAX(int canID, MechanismRatio ratio) {
    this(canID, ratio, makeDefaultConfig());
  }

  public QuixSparkMAX(int canID, MechanismRatio ratio, boolean inverted) {
    this(canID, ratio, makeDefaultConfig().setInverted(inverted));
  }

  /** Follower constructor */
  public QuixSparkMAX(int canID, QuixSparkMAX leader) {
    this(canID, new MechanismRatio()); // Mechanism ratio for a follower is not used.
    m_controller.follow(leader.m_controller);
  }

  /** Constructor with full configuration */
  public QuixSparkMAX(int canID, MechanismRatio ratio, QuixSparkMAXConfiguration config) {
    m_controller = new CANSparkMax(canID, MotorType.kBrushless);
    m_encoder = m_controller.getEncoder();
    m_pid = m_controller.getPIDController();
    m_ratio = ratio;
    m_config = config;

    // Set motor controller configuration.
    m_controller.restoreFactoryDefaults();
    m_controller.setIdleMode(config.IDLE_MODE);
    if (config.ENABLE_VOLTAGE_COMPENSATION) {
      m_controller.enableVoltageCompensation(config.VOLTAGE_COMPENSATION_SATURATION);
    } else {
      m_controller.disableVoltageCompensation();
    }
    m_controller.setInverted(config.INVERTED);
    if (config.ENABLE_CURRENT_LIMIT) {
      m_controller.setSmartCurrentLimit(config.CURRENT_LIMIT);
    }

    // Reduce rate of unnecessary CAN frames.
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

    // Write all settings to flash to be extra safe.
    // Flash has limited write cycles, so only do this on FMS.
    if (DriverStation.isFMSAttached()) {
      m_controller.burnFlash();
    }
  }

  public void close() {
    m_controller.close();
  }

  public int getDeviceID() {
    return m_controller.getDeviceId();
  }

  public void setPercentOutput(double percent) {
    m_controller.set(percent);
  }

  public void setVoltageOutput(double voltage) {
    m_controller.setVoltage(voltage);
  }

  public void setPIDConfig(int slot, PIDConfig config) {
    // TODO: convert gains to spark units for ease of tuning.
    m_pid.setP(config.getP(), slot);
    m_pid.setI(config.getI(), slot);
    m_pid.setD(config.getD(), slot);
    m_pid.setFF(config.getF(), slot);
    // TODO: burn flash?
  }

  public void setPositionSetpoint(int slot, double setpoint) {
    setPositionSetpoint(slot, setpoint, 0.0);
  }

  public void setPositionSetpoint(int slot, double setpoint, double feedforwardVolts) {
    m_pid.setReference(
        toNativeSensorPosition(setpoint),
        CANSparkMax.ControlType.kPosition,
        slot,
        feedforwardVolts,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public void setVelocitySetpoint(int slot, double setpoint) {
    setVelocitySetpoint(slot, setpoint, 0.0);
  }

  public void setVelocitySetpoint(int slot, double setpoint, double feedforwardVolts) {
    m_pid.setReference(
        toNativeSensorVelocity(setpoint),
        CANSparkMax.ControlType.kVelocity,
        slot,
        feedforwardVolts,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public double getMaxVoltage() {
    return m_config.ENABLE_VOLTAGE_COMPENSATION
        ? m_config.VOLTAGE_COMPENSATION_SATURATION
        : RobotController.getBatteryVoltage();
  }

  public double getPercentOutput() {
    return m_controller.getAppliedOutput();
  }

  public double getPhysicalPercentOutput() {
    return (getInverted() ? -1.0 : 1.0) * m_controller.getAppliedOutput();
  }

  public double getVoltageOutput() {
    return m_controller.getBusVoltage() * m_controller.getAppliedOutput();
  }

  public boolean getInverted() {
    return m_controller.getInverted();
  }

  public void zeroSensorPosition() {
    setSensorPosition(0.0);
  }

  public void setSensorPosition(double pos) {
    m_encoder.setPosition(toNativeSensorPosition(pos));
  }

  public double getSensorPosition() {
    return fromNativeSensorPosition(m_encoder.getPosition());
  }

  public double getSensorVelocity() {
    return fromNativeSensorVelocity(m_encoder.getVelocity());
  }

  public MechanismRatio getMechanismRatio() {
    return m_ratio;
  }

  public double toNativeSensorPosition(double pos) {
    final double motorRadians = m_ratio.mechanismPositionToSensorRadians(pos);
    // Native position is revolutions.
    return motorRadians / (2.0 * Math.PI);
  }

  public double fromNativeSensorPosition(double pos) {
    return pos / toNativeSensorPosition(1.0);
  }

  public double toNativeSensorVelocity(double vel) {
    // Native velocity is revolutions per minute.
    return toNativeSensorPosition(vel) * 60.0;
  }

  public double fromNativeSensorVelocity(double vel) {
    return vel / toNativeSensorVelocity(1.0);
  }

  public void setSimSensorPositionAndVelocity(
      double pos, double vel, double dt, MechanismRatio mr) {
    // TODO: REV has its own simulation. Figure out how to use it.
    return;
  }

  public void setSimSensorVelocity(double vel, double dt, MechanismRatio mr) {
    // TODO: REV has its own simulation. Figure out how to use it.
    return;
  }
}
