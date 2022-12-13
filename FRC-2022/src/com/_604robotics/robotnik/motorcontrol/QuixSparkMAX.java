package com._604robotics.robotnik.motorcontrol;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class QuixSparkMAX extends QuixMotorController {

  public CANSparkMax controller;

  private String name;

  public QuixSparkMAX(int port, String name, Motor motor, SubsystemBase subsystem) {
    super(motor, subsystem);
    if (super.motor == Motor.kNEO || super.motor == Motor.kNEO550) {
      controller = new CANSparkMax(port, MotorType.kBrushless);
    } else {
      controller = new CANSparkMax(port, MotorType.kBrushed);
    }

    // Reduce rate of unnecessary CAN frames
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);

    controller.enableVoltageCompensation(12);

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

    if (isCurrentLimiting()) {
      controller.setSmartCurrentLimit(getCurrentLimit());
    }
  }

  public void setVoltageCompSaturation(double volts, boolean enable) {
    if (enable) {
      controller.enableVoltageCompensation(volts);
    } else {
      controller.disableVoltageCompensation();
    }
  }

  public void follow(QuixSparkMAX master, boolean inverted) {
    controller.follow(master.controller, inverted);
    // Reduce status frames for follower.
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    controller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
  }

  @Override
  public double getOutputCurrent() {
    return controller.getOutputCurrent();
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

  public void burnFlashConditionally(boolean force) {
    if (DriverStation.isFMSAttached() || force) {
      controller.burnFlash();
    }
  }

  public void resetParams() {
    controller.restoreFactoryDefaults();
  }

  @Override
  public double getInputVoltage() {
    return controller.getBusVoltage();
  }
}
