package com._604robotics.robotnik.controller;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of PIDController that has a feedforward for rotating arms. This subclass requires an
 * PIDSource as the PIDSource and uses only continuous error. Zero is assumed to be horizontal.
 * Users are responsible for properly zeroing the PIDSource beforehand.
 */
public class ProfiledRotatingArmPIDController extends ProfiledPIDController {
  private ArmFeedforward feedforward;
  private double encoderPeriod = 360;
  private double zeroOffset = 0;

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, constraints, source, output);
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, constraints, period, source, output);
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      ArmFeedforward feedforward,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, constraints, source, output);
    this.feedforward = feedforward;
  }

  public ProfiledRotatingArmPIDController(
      double Kp,
      double Ki,
      double Kd,
      ArmFeedforward feedforward,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, constraints, period, source, output);
    this.feedforward = feedforward;
  }

  public double getEncoderPeriod() {
    return encoderPeriod;
  }

  public void setEncoderPeriod(double encoderPeriod) {
    this.encoderPeriod = encoderPeriod;
  }

  public void setFeedforwardZeroOffset(double zeroOffset) {
    this.zeroOffset = zeroOffset;
  }

  /**
   * Overridden feed forward part of ProfiledPIDController. This is a physically based model which
   * multiplies feed forward coefficient by cosine. The feedforward calculates the expected torque
   * needed to hold an arm steady, scaled to motor power.
   *
   * @return the feed forward value
   */
  @Override
  protected double calculateFeedForward(double setpoint) {
    // Calculate cosine for torque factor
    double angle;
    thisMutex.lock();
    try {
      angle = pidSource.getAsDouble() - zeroOffset;
    } finally {
      thisMutex.unlock();
    }

    // Cosine is periodic so sawtooth wraparound is not a concern
    angle /= encoderPeriod;
    angle *= 360;

    /*
     Dividing by 12 to convert volts to motor output power. We can ignore the current voltage of the battery as voltage saturation is available on talons.
    */
    return feedforward.calculate(angle, 0) / 12;
  }
}
