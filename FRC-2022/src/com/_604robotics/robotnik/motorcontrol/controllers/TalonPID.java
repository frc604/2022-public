package com._604robotics.robotnik.motorcontrol.controllers;

import com._604robotics.robotnik.devices.FalconEncoder;
import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TalonPID extends MotorControllerPID{
  public WPI_TalonFX controller;
  private FalconEncoder encoder;

  private int slot;

  /**
   * A wrapper representing a PID controller running on a REV Robotics SparkMAX.
   *
   * @param spark The QuixSparkMAX({@link
   *     com._604robotics.robotnik.motorcontrol.QuixSparkMAX}) object that the controller is
   *     running on.
   * @param Kp The initial proportional gain.
   * @param Ki The initial integral gain.
   * @param KD The initial derivative gain.
   */
  public TalonPID(QuixTalonFX talon, FalconEncoder encoder, MotorControllerPIDConfig config) {
    super(talon, encoder, config);
    this.controller = talon.getController();
    this.encoder = encoder;
    this.slot = config.slot;

    // this.controller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    this.setConfig(config);
  }

  @Override
  public void setSetpointVelocity(double setpoint) {
    this.controller.selectProfileSlot(slot, 0);
    this.controller.set(
      ControlMode.Velocity,  (setpoint / encoder.getPositionConversionFactor()) * (1.0 / 10.0), DemandType.ArbitraryFeedForward, 0.0);
  }

  @Override
  public void setSetpointVelocity(double setpoint, double feedforwardVolts) {
    this.controller.selectProfileSlot(slot, 0);
    this.controller.set(
      ControlMode.Velocity,
        (setpoint / encoder.getPositionConversionFactor()) * (1.0 / 10.0),
        DemandType.ArbitraryFeedForward,
        feedforwardVolts / 12.0);
  }

  @Override
  public void setSetpointPosition(double setpoint) {
    this.controller.selectProfileSlot(slot, 0);
    this.controller.set(ControlMode.Position, (setpoint / encoder.getPositionConversionFactor()));
    // System.out.println("Falcon: " + Conversions.degreesToFalcon(setpoint, 12.8));
    // System.out.println("Conversion: " + (setpoint / encoder.getPositionConversionFactor()));
  }

  @Override
  public void setSetpointPosition(double setpoint, double feedforwardVolts) {
    this.controller.selectProfileSlot(slot, 0);
    this.controller.set(ControlMode.Position, (setpoint / encoder.getPositionConversionFactor()), DemandType.ArbitraryFeedForward, feedforwardVolts / 12.0);
  }

  @Override
  public void setConfig(MotorControllerPIDConfig config) {
    this.controller.config_kP(config.slot, config.Kp);
    this.controller.config_kI(config.slot, config.Ki);
    this.controller.config_kD(config.slot, config.Kd);
    this.controller.config_kF(config.slot, config.Kf);
  }
}
