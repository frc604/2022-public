package com._604robotics.robotnik.motorcontrol.controllers;

import com._604robotics.robotnik.devices.IntegratedEncoder;
import com._604robotics.robotnik.motorcontrol.QuixMotorController;

public abstract class MotorControllerPID {
  protected QuixMotorController controller;
  protected IntegratedEncoder encoder;
  protected MotorControllerPIDConfig config;

  public MotorControllerPID(QuixMotorController controller, IntegratedEncoder encoder, MotorControllerPIDConfig config) {
    this.controller = controller;
    this.encoder = encoder;
    this.config = config;
  }

  public void setSetpointVelocity(double setpoint) {};

  public void setSetpointPosition(double setpoint) {};

  public void setSetpointVelocity(double setpoint, double feedforwardVolts) {};

  public void setSetpointPosition(double setpoint, double feedforwardVolts) {};

  public void setConfig(MotorControllerPIDConfig config) {};
}
