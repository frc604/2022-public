package com._604robotics.robotnik.controller;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * An extension of {@link ExtendablePIDController} that uses a WPILib trapezoidal profile {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} in the output calculation.
 */
public class ProfiledPIDController extends ExtendablePIDController {
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  // Motion Profile constraints.
  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.0, 0.0);

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, and motion profile
   * constraints and a default period of 0.02 seconds with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints The motion profile constraints.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, constraints, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, motion profile
   * constraints, and period with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param constraints The motion profile constraints.
   * @param period The period of the controller.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      TrapezoidProfile.Constraints constraints,
      double period,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, constraints, period, pidSource, pidOutput);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, Kf, and motion
   * profile constraints and a default period of 0.02 seconds.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param constraints The motion profile constraints.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, Kf, constraints, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, Kd, Kf, motion profile
   * constraints, and peroid.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param constraints The motion profile constraints.
   * @param period The period of the controller in seconds.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidSource A lambda consuming the controller output.
   */
  public ProfiledPIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      TrapezoidProfile.Constraints constraints,
      double period,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {

    super(Kp, Ki, Kd, Kf, period, pidSource, pidOutput);
    this.constraints = constraints;
  }

  /**
   * Sets the goal for the PIDController.
   *
   * @param goal The goal position.
   */
  public void setGoal(double goal) {
    thisMutex.lock();
    try {
      this.goal = new TrapezoidProfile.State(goal, 0);
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public TrapezoidProfile.State getGoal() {
    thisMutex.lock();
    try {
      return goal;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Returns true if the error is within the percentage of the total input range, determined by
   * SetTolerance. This assumes that the maximum and minimum input were set using SetInput.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atGoal() {
    thisMutex.lock();
    try {
      return atSetpoint() && goal.equals(setpoint);
    } finally {
      thisMutex.unlock();
    }
  }

  public void setInitialState(TrapezoidProfile.State state) {
    thisMutex.lock();
    try {
      setpoint = state;
    } finally {
      thisMutex.unlock();
    }
  }

  public void setInitialSetpoint(double setpoint) {
    setInitialState(new TrapezoidProfile.State(setpoint, 0.0));
  }

  /**
   * Sets the constraints for the controller motion profile.
   *
   * @param constraints The constraints
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    thisMutex.lock();
    try {
      this.constraints = constraints;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Returns the set constraints for the controller motion profile.
   *
   * @return The motion profile constraints.
   */
  public TrapezoidProfile.Constraints getConstraints() {
    thisMutex.lock();
    try {
      return constraints;
    } finally {
      thisMutex.unlock();
    }
  }

  @Override
  protected void calculate() {
    if (origSource == null || pidOutput == null) {
      return;
    }

    boolean enabled;

    thisMutex.lock();
    try {
      enabled = this.enabled;
    } finally {
      thisMutex.unlock();
    }

    if (enabled) {
      double input;

      double P;
      double I;
      double D;

      double period;

      double maximumIntegral;
      double minimumIntegral;

      double maximumOutput;
      double minimumOutput;

      TrapezoidProfile profile;
      TrapezoidProfile.State setpoint;

      // Storage for function input-outputs
      double positionError;
      double velocityError;
      double totalError;

      thisMutex.lock();
      try {
        input = pidSource.getAsDouble();

        P = Kp;
        I = Ki;
        D = Kd;

        period = this.period;

        maximumIntegral = this.maximumIntegral;
        minimumIntegral = this.minimumIntegral;

        maximumOutput = this.maximumOutput;
        minimumOutput = this.minimumOutput;

        totalError = this.totalError;

        // Making the motion profile and finding the controller setpoint.
        profile = new TrapezoidProfile(constraints, goal, this.setpoint);
        setpoint = profile.calculate(period);

        positionError = getContinuousError(setpoint.position - input);
        velocityError = (positionError - prevError) / period;

      } finally {
        thisMutex.unlock();
      }

      // Storage for function outputs
      double result;

      // Clamping the Integral coefficient
      if (I != 0) {
        totalError =
            MathUtil.clamp(
                totalError + positionError * period, minimumIntegral / I, maximumIntegral / I);
      }

      // Calculating the result
      result =
          calculateProportional(P, positionError)
              + calculateIntegral(I, totalError)
              + calculateDerivative(D, velocityError)
              + calculateFeedForward(setpoint.velocity);

      // Clamping the result
      result = MathUtil.clamp(result, minimumOutput, maximumOutput);

      // Ensures enabled check and pidWrite() call occur atomically
      pidOutputMutex.lock();
      try {
        thisMutex.lock();
        try {
          if (enabled) {
            // Don't block other PIDController operations on pidWrite()
            thisMutex.unlock();

            pidOutput.accept(result);
          }
        } finally {
          if (thisMutex.isHeldByCurrentThread()) {
            thisMutex.unlock();
          }
        }
      } finally {
        pidOutputMutex.unlock();
      }

      thisMutex.lock();
      try {
        prevError = positionError;
        this.totalError = totalError;
        this.setpoint = setpoint;
        this.result = result;
        this.positionError = positionError;
        this.velocityError = velocityError;
      } finally {
        thisMutex.unlock();
      }
    }
  }

  /**
   * Sets the minimum and maximum values expected from the input.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  @Override
  public void setInputRange(double minimumInput, double maximumInput) {
    this.thisMutex.lock();
    try {
      this.minimumInput = minimumInput;
      this.maximumInput = maximumInput;
      inputRange = maximumInput - minimumInput;

      // Clamp setpoint to new input
      if (this.maximumInput > this.minimumInput) {
        setpoint =
            new TrapezoidProfile.State(
                MathUtil.clamp(setpoint.position, minimumInput, maximumInput), setpoint.velocity);
      }

    } finally {
      thisMutex.unlock();
    }
  }

  @Override
  public double getSetpoint() {
    throw new UnsupportedOperationException("Should not be accessed by, use getGoal() instead!");
  }

  @Override
  public void setSetpoint(double setpoint) {
    throw new UnsupportedOperationException("Should not be accessed by, use setGoal() instead!");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.setSafeState(this::reset);
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("f", this::getF, this::setF);
    builder.addDoubleProperty("goal", () -> getGoal().position, this::setGoal);
    builder.addDoubleProperty(
        "maxVelocity",
        () -> getConstraints().maxVelocity,
        (maxVelocity) ->
            setConstraints(
                new TrapezoidProfile.Constraints(maxVelocity, getConstraints().maxAcceleration)));
    builder.addDoubleProperty(
        "maxAcceleration",
        () -> getConstraints().maxAcceleration,
        (maxAcceleration) ->
            setConstraints(
                new TrapezoidProfile.Constraints(getConstraints().maxVelocity, maxAcceleration)));
    builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
    builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
  }
}
