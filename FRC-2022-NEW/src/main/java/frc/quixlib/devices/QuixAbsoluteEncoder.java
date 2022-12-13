package frc.quixlib.devices;

public interface QuixAbsoluteEncoder {
  // ==================== Setters ====================

  /** Sets the sensor zero-point to the current position. */
  public void zero();

  /** Sets the sensor position to the given value. Uses MechanismRatio units. */
  public void setPosition(double pos);

  // ==================== Getters ====================

  /** Returns the sensor position. Uses MechanismRatio units. */
  public double getPosition();

  /** Returns the absolute sensor position. Uses MechanismRatio units. */
  public double getAbsPosition();

  /** Returns the sensor velocity. Uses MechanismRatio units. */
  public double getVelocity();

  // ==================== Unit Conversions ====================

  /** Convert MechanismRatio position to native sensor position. */
  public double toNativeSensorPosition(double pos);

  /** Convert from native sensor position. Returns position in MechanismRatio units. */
  public double fromNativeSensorPosition(double pos);

  /** Convert MechanismRatio velocity to native sensor velocity. */
  public double toNativeSensorVelocity(double vel);

  /** Convert from native sensor velocity. Returns velocity in MechanismRatio units. */
  public double fromNativeSensorVelocity(double vel);

  // ==================== Simulation ====================

  /**
   * Sets the simulated angular velocity of the sensor in mechanism units. Also sets the simulated
   * position.
   */
  public void setSimSensorVelocity(double vel, double dt);
}
