package frc.quixlib.motorcontrol;

/** Defines a ratio and distance per rotation between motor and mechanism. */
public class MechanismRatio {
  private double driving = 1.0;
  private double driven = 1.0;
  private double distancePerRotation = 2.0 * Math.PI; // Default to radians

  public MechanismRatio() {}

  public MechanismRatio(double drivingTeeth, double drivenTeeth) {
    this.driving = drivingTeeth;
    this.driven = drivenTeeth;
  }

  public MechanismRatio(double drivingTeeth, double drivenTeeth, double distancePerRotation) {
    this.driving = drivingTeeth;
    this.driven = drivenTeeth;
    this.distancePerRotation = distancePerRotation;
  }

  /**
   * Returns the mechanism ratio defined as: driven / driving. Numbers greater than 1 represent
   * reductions.
   */
  public double reduction() {
    return driven / driving;
  }

  /** Returns the mechanism position for the given sensor position in radians. */
  public double sensorRadiansToMechanismPosition(double sensorRadians) {
    final double distancePerRadian = distancePerRotation / (2.0 * Math.PI);
    return sensorRadians * distancePerRadian / reduction();
  }

  /** Returns the sensor position in radians for the given mechanism position. */
  public double mechanismPositionToSensorRadians(double mechanismPos) {
    return mechanismPos / sensorRadiansToMechanismPosition(1.0);
  }
}
