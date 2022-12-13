package com._604robotics.robotnik.controller;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Subclass of PIDController that has a feedforward for auto aligning of a limelight angle. This
 * subclass requires an double as the PIDSource and uses only continuous error.
 */
public class AutoTargetPIDController extends ProfiledPIDController {
  private SimpleMotorFeedforward feedforward;

  public AutoTargetPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      SimpleMotorFeedforward feedforward,
      DoubleSupplier source,
      DoubleConsumer output) {
    super(Kp, Ki, Kd, constraints, source, output);

    this.feedforward = feedforward;
  }

  public AutoTargetPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      SimpleMotorFeedforward feedforward,
      DoubleSupplier source,
      DoubleConsumer output,
      double period) {
    super(Kp, Ki, Kd, constraints, period, source, output);

    this.feedforward = feedforward;
  }

  /**
   * Overridden feed forward part of PIDController. This is a physically based model which
   * multiplies feed forward coefficient by cosine. The feedforward calculates the expected torque
   * needed to hold an arm steady, scaled to motor power.
   *
   * @return the feed forward value
   */
  @Override
  protected double calculateFeedForward(double setpoint) {
    return feedforward.calculate(setpoint);
  }
}
