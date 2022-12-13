package com._604robotics.robotnik.motorcontrol;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuixTalonSRX extends QuixMotorController {

  public WPI_TalonSRX controller;

  private String name;

  private int peakLimit = 0;

  public QuixTalonSRX(int port, String name, SubsystemBase subsystem) {
    super(subsystem);
    controller = new WPI_TalonSRX(port);
    controller.configVoltageCompSaturation(12);
    controller.enableVoltageCompensation(true);

    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
    controller.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);

    PowerMonitor.getInstance().addController(this, name);
  }

  public String getName() {
    return name;
  }

  @Override
  public void set(double power) {
    controller.set(power);
  }

  @Override
  public void enableCurrentLimit(boolean enable) {
    super.isLimiting = enable;

    controller.enableCurrentLimit(enable);

    if (isCurrentLimiting()) {
      controller.configContinuousCurrentLimit(getCurrentLimit());
      controller.configPeakCurrentLimit(peakLimit);
    }
  }

  public void follow(QuixTalonSRX master, boolean inverted) {
    controller.follow(master.controller);

    if (inverted) {
      setInverted(InvertType.OpposeMaster);
    }
  }

  public void setPeakCurrentLimit(int peakLimit) {
    this.peakLimit = peakLimit;
  }

  public void setVoltageCompSaturation(double volts, boolean enable) {
    controller.configVoltageCompSaturation(volts);
    controller.enableVoltageCompensation(enable);
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

  public void setInverted(InvertType invertType) {
    controller.setInverted(invertType);
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
    set(0.0);
  }

  @Override
  public double getOutputVoltage() {
    return controller.get() * getInputVoltage();
  }

  @Override
  public double getInputVoltage() {
    return controller.getBusVoltage();
  }
}
