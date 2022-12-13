package frc.quixlib.motorcontrol;

interface QuixMotorControllerWithEncoder {
  /** Returns the CAN ID of the device. */
  public int getDeviceID();

  // ==================== Motor Setters ====================

  /** Sets percent output from -1.0 to 1.0. */
  public void setPercentOutput(double percent);

  /** Sets voltage output. */
  public void setVoltageOutput(double voltage);

  /** Set the PID gains for the specified slot. */
  public void setPIDConfig(int slot, PIDConfig config);

  /** Closed-loop position mode. Position setpoint defined in MechanismRatio units. */
  public void setPositionSetpoint(int slot, double setpoint);

  /**
   * Closed-loop position mode with feed-forward. Position setpoint defined in MechanismRatio units.
   */
  public void setPositionSetpoint(int slot, double setpoint, double feedforwardVolts);

  /** Closed-loop velocity mode. Velocity setpoint defined in MechanismRatio units. */
  public void setVelocitySetpoint(int slot, double setpoint);

  /**
   * Closed-loop velocity mode with feed-forward. Velocity setpoint defined in MechanismRatio units.
   */
  public void setVelocitySetpoint(int slot, double setpoint, double feedforwardVolts);

  // ==================== Motor Getters ====================

  /** Returns the max volage corresponding to 100% output. */
  public double getMaxVoltage();

  /** Returns the percent output (-1.0 to 1.0) as reported by the device. */
  public double getPercentOutput();

  /**
   * Returns the percent output (-1.0 to 1.0) as reported by the device. Sign corresponds to the
   * physical direction of the motor. Useful for simulation.
   */
  public double getPhysicalPercentOutput();

  /** Returns the applied voltage to the motor in volts. Same as busVoltage * percentOutput */
  public double getVoltageOutput();

  /** Returns whether the motor is inverted. */
  public boolean getInverted();

  // ==================== Sensor Setters ====================

  /** Sets the sensor zero-point to the current position. */
  public void zeroSensorPosition();

  /** Sets the sensor position to the given value. Uses MechanismRatio units. */
  public void setSensorPosition(double pos);

  // ==================== Sensor Getters ====================

  /** Returns the sensor position. Uses MechanismRatio units. */
  public double getSensorPosition();

  /** Returns the sensor velocity. Uses MechanismRatio units. */
  public double getSensorVelocity();

  // ==================== Unit Conversions ====================

  /** Returns the MechaismRatio. */
  public MechanismRatio getMechanismRatio();

  /** Convert MechanismRatio position to native sensor position. */
  public double toNativeSensorPosition(double pos);

  /** Convert from native sensor position. Returns position in MechanismRatio units. */
  public double fromNativeSensorPosition(double pos);

  /** Convert MechanismRatio velocity to native sensor velocity. */
  public double toNativeSensorVelocity(double vel);

  /** Convert from native sensor velocity. Returns velocity in MechanismRatio units. */
  public double fromNativeSensorVelocity(double vel);

  // ==================== Simulation ====================

  /** Sets the simulated angular position and velocity of the sensor in mechanism units. */
  public void setSimSensorPositionAndVelocity(double pos, double vel, double dt, MechanismRatio mr);

  /**
   * Sets the simulated angular velocity of the sensor in mechanism units. Also sets the simulated
   * position.
   */
  public void setSimSensorVelocity(double vel, double dt, MechanismRatio mr);
}
