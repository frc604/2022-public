package com._604robotics.robotnik.motorcontrol;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuixTalonFX extends QuixMotorController {

  private WPI_TalonFX controller;
  private TalonFXSimCollection sim_collection;

  private String name;

  private double peakCurrentLimit = 40;
  private double peakCurrentDuration = 0.1;

  public QuixTalonFX(int port, String name, Motor motor, SubsystemBase subsystem) {
    super(motor, subsystem);

    this.controller = new WPI_TalonFX(port);
    this.sim_collection = this.controller.getSimCollection();

    controller.configFactoryDefault();

    controller.enableVoltageCompensation(true);
    controller.configVoltageCompSaturation(12);

    controller.configNeutralDeadband(0.0);

    PowerMonitor.getInstance().addController(this, name);

    // Reduce rate of unnecessary CAN frames
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 500);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 1000);
  }

  public String getName() {
    return name;
  }

  public WPI_TalonFX getController() {
    return controller;
  }

  public TalonFXSimCollection getSimCollection() {
    return sim_collection;
  }

  @Override
  public void set(double power) {
    controller.set(power);
  }

  @Override
  public void enableCurrentLimit(boolean enable) {
    super.isLimiting = enable;

    controller.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(enable, getCurrentLimit(), this.peakCurrentLimit, this.peakCurrentDuration));
  }

  public void setVoltageCompSaturation(double volts, boolean enable) {
    controller.configVoltageCompSaturation(volts);
    controller.enableVoltageCompensation(enable);
  }

  public void follow(QuixTalonFX master, boolean inverted) {
    controller.follow(master.controller);
  }

  @Override
  public double getOutputCurrent() {
    return controller.getSupplyCurrent();
  }

  @Override
  public double get() {
    return controller.get();
  }

  @Override
  public void setInverted(boolean inverted) {
    controller.setInverted(inverted);
  }

  @Override
  public boolean getInverted() {
    return controller.getInverted();
  }

  @Override
  public void disable() {
    controller.disable();
  }

  @Override
  public void stopMotor() {
    controller.stopMotor();
  }

  @Override
  public double getOutputVoltage() {
    return controller.get() * getInputVoltage();
  }

  @Override
  public double getInputVoltage() {
    return controller.getBusVoltage();
  }

  public void setPeakCurrentLimit(double limit) {
    this.peakCurrentLimit = limit;
  }

  public double getPeakCurrentLimit() {
    return this.peakCurrentLimit;
  }

  public void setPeakCurrentDuration(double duration) {
    this.peakCurrentDuration = duration;
  }

  public double getPeakCurrentDuration() {
    return this.peakCurrentDuration;
  }
}
