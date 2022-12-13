package com._604robotics.robotnik.motorcontrol;


import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class QuixMotorController implements MotorController {

  protected final Motor motor;

  protected final SubsystemBase subsystem;

  protected int currentLimit = 0;

  protected boolean isLimiting = false;

  public QuixMotorController(Motor motor, SubsystemBase subsystem) {
    this.motor = motor;
    this.subsystem = subsystem;
  }

  public QuixMotorController(SubsystemBase subsystem) {
    this.motor = null;
    this.subsystem = subsystem;
  }

  public abstract void set(double power);

  public abstract double get();

  public abstract double getOutputVoltage();

  public abstract double getInputVoltage();

  public abstract double getOutputCurrent();

  public abstract void setInverted(boolean inverted);

  public abstract boolean getInverted();

  public abstract void disable();

  public abstract void stopMotor();

  public Motor getMotor() {
    return motor;
  }

  public void setCurrentLimit(int limit) {
    currentLimit = limit;
  }

  public void enableCurrentLimit(boolean enable) {
    isLimiting = enable;
  }

  public int getCurrentLimit() {
    return currentLimit;
  }

  public boolean isCurrentLimiting() {
    return isLimiting;
  }

  public SubsystemBase getSubsystem() {
    
    return subsystem;
  }
}
