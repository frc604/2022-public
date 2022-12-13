/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* This is 604's extendable version which has been                            */
/* re-written for the 2020 season                                             */
/*----------------------------------------------------------------------------*/

package com._604robotics.robotnik.controller;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.util.BoundaryException;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.MathUtil;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/** Implements a PID control loop. */
public class ExtendablePIDController implements Sendable, AutoCloseable {
  private static int instances;

  // Factor for "proportional" control
  protected double Kp;

  // Factor for "integral" control
  protected double Ki;

  // Factor for "derivative" control
  protected double Kd;

  // Factor for "feedforward" term
  protected double Kf;

  // The period (in seconds) of the loop that calls the controller
  protected final double period;

  // Maximum integral value
  protected double maximumIntegral = 1.0;

  // Minimum integral value
  protected double minimumIntegral = -1.0;

  // Maximum input - limit setpoint to this
  protected double maximumInput;

  // Minimum input - limit setpoint to this
  protected double minimumInput;

  protected double maximumOutput = 1;
  protected double minimumOutput = -1;

  // Input range - difference between maximum and minimum
  protected double inputRange;

  // Do the endpoints wrap around? eg. Absolute encoder
  protected boolean continuous;

  // Is the pid controller enabled
  protected boolean enabled = false;

  // The error at the time of the most recent call to calculate()
  protected double positionError;
  protected double velocityError;

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  protected double prevError;

  // The sum of the errors for use in the integral calc
  protected double totalError;

  // The percentage or absolute error that is considered at setpoint.
  protected double positionTolerance = Double.POSITIVE_INFINITY;
  protected double velocityTolerance = Double.POSITIVE_INFINITY;

  // Supplied setpoint
  private double setpoint;

  // Last measurement used for calculation
  protected double prevMeasurement;

  // Last calculated result of the controller
  protected double result = 0.0;

  // Output consumer.
  protected DoubleConsumer pidOutput;

  // Source supplier.
  protected DoubleSupplier pidSource;

  // Original source of the controller, if it changes.
  protected DoubleSupplier origSource;

  // Thread lock for the controller.
  ReentrantLock thisMutex = new ReentrantLock();

  // Thread lock for writing to the output.
  ReentrantLock pidOutputMutex = new ReentrantLock();

  // Control loop Notifier.
  Notifier notifier;

  /**
   * Allocates a ExtendablePIDController with the given constants for Kp, Ki, and Kd and a default
   * period of 0.02 seconds with no feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ExtendablePIDController(
      double Kp, double Ki, double Kd, DoubleSupplier pidSource, DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, 0.0, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a ExtendablePIDController with the given constants for Kp, Ki, kD, and period with no
   * feedforward.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   * @param period The period of the controller in seconds.
   */
  public ExtendablePIDController(
      double Kp,
      double Ki,
      double Kd,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput,
      double period) {
    this(Kp, Ki, Kd, 0.0, period, pidSource, pidOutput);
  }

  /**
   * Allocates a ExtendablePIDController with the given constants for Kp, Ki, Kd, and Kf and a
   * default period of 0.02 seconds.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ExtendablePIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {
    this(Kp, Ki, Kd, Kf, 0.02, pidSource, pidOutput);
  }

  /**
   * Allocates a ExtendablePIDController with the given constants for Kp, Ki, Kd, Kf, and period.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   * @param Kf The feedforward term.
   * @param period The period of the controller in seconds.
   * @param pidSource A lambda supplying the controller measurement.
   * @param pidOutput A lambda consuming the controller output.
   */
  public ExtendablePIDController(
      double Kp,
      double Ki,
      double Kd,
      double Kf,
      double period,
      DoubleSupplier pidSource,
      DoubleConsumer pidOutput) {

    // Making sure null supplier and consumers are not passed in.
    requireNonNullParam(pidSource, "pidSource", "Null PIDSource was given");
    requireNonNullParam(pidOutput, "pidOutput", "Null PIDOutput was given");

    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;

    this.period = period;

    this.pidSource = pidSource;
    this.pidOutput = pidOutput;

    origSource = pidSource;

    prevMeasurement = pidSource.getAsDouble();

    notifier = new Notifier(this::calculate);
    notifier.startPeriodic(period);

    // Adding instance of PIDController
    instances++;
    SendableRegistry.addLW(this, "PIDController", instances);

    HAL.report(tResourceType.kResourceType_PIDController, instances);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
    notifier.close();

    thisMutex.lock();
    try {
      pidOutput = null;
      pidSource = null;
      notifier = null;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the PID Controller gain parameters.
   *
   * <p>Set the proportional, integral, and differential coefficients.
   *
   * @param Kp The proportional coefficient.
   * @param Ki The integral coefficient.
   * @param Kd The derivative coefficient.
   */
  @SuppressWarnings("ParameterName")
  public void setPID(double Kp, double Ki, double Kd) {
    thisMutex.lock();
    try {
      this.Kp = Kp;
      this.Ki = Ki;
      this.Kd = Kd;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the Proportional coefficient of the PID controller gain.
   *
   * @param Kp proportional coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setP(double Kp) {
    thisMutex.lock();
    try {
      this.Kp = Kp;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the Integral coefficient of the PID controller gain.
   *
   * @param Ki integral coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setI(double Ki) {
    thisMutex.lock();
    try {
      this.Ki = Ki;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the Differential coefficient of the PID controller gain.
   *
   * @param Kd differential coefficient
   */
  @SuppressWarnings("ParameterName")
  public void setD(double Kd) {
    thisMutex.lock();
    try {
      this.Kd = Kd;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the Feedforward term of the PID controller.
   *
   * @param Kf feedforward term
   */
  @SuppressWarnings("ParameterName")
  public void setF(double Kf) {
    thisMutex.lock();
    try {
      this.Kf = Kf;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Get the Proportional coefficient.
   *
   * @return proportional coefficient
   */
  public double getP() {
    thisMutex.lock();
    try {
      return Kp;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Get the Integral coefficient.
   *
   * @return integral coefficient
   */
  public double getI() {
    thisMutex.lock();
    try {
      return Ki;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Get the Differential coefficient.
   *
   * @return differential coefficient
   */
  public double getD() {
    thisMutex.lock();
    try {
      return Kd;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Get the Feedforward term.
   *
   * @return feedforward term
   */
  public double getF() {
    thisMutex.lock();
    try {
      return Kf;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Returns the period of this controller.
   *
   * @return the period of the controller.
   */
  public double getPeriod() {
    thisMutex.lock();
    try {
      return period;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  public void setSetpoint(double setpoint) {
    thisMutex.lock();
    try {
      if (maximumInput > minimumInput) {
        this.setpoint = MathUtil.clamp(setpoint, minimumInput, maximumInput);
      } else {
        this.setpoint = setpoint;
      }
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    thisMutex.lock();
    try {
      return setpoint;
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
  public boolean atSetpoint() {
    thisMutex.lock();
    try {
      return Math.abs(positionError) < positionTolerance
          && Math.abs(velocityError) < velocityTolerance;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    thisMutex.lock();
    try {
      continuous = true;
      setInputRange(minimumInput, maximumInput);
    } finally {
      thisMutex.unlock();
    }
  }

  /** Disables continuous input. */
  public void disableContinuousInput() {
    thisMutex.lock();
    try {
      continuous = false;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values to write.
   *
   * @param minimumOutput the minimum percentage to write to the output
   * @param maximumOutput the maximum percentage to write to the output
   */
  public void setOutputRange(double minimumOutput, double maximumOutput) {
    thisMutex.lock();
    try {
      if (minimumOutput > maximumOutput) {
        throw new BoundaryException("Lower bound is greater than upper bound");
      }
      this.minimumOutput = minimumOutput;
      this.maximumOutput = maximumOutput;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * <p>When the cap is reached, the integrator value is added to the controller output rather than
   * the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
    thisMutex.lock();
    try {
      this.minimumIntegral = minimumIntegral;
      this.maximumIntegral = maximumIntegral;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   */
  public void setTolerance(double positionTolerance) {
    setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Sets the error which is considered tolerable for use with atSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    thisMutex.lock();
    try {
      this.positionTolerance = positionTolerance;
      this.velocityTolerance = velocityTolerance;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    thisMutex.lock();
    try {
      return getContinuousError(positionError);
    } finally {
      thisMutex.unlock();
    }
  }

  /** Returns the velocity error. */
  public double getVelocityError() {
    thisMutex.lock();
    try {
      return velocityError;
    } finally {
      thisMutex.unlock();
    }
  }

  protected void calculate() {
    if (origSource == null || pidOutput == null) {
      return;
    }

    boolean enabled;

    // Checking if controller is enabled or not.
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

      // Storage for function input-outputs
      double positionError;
      double velocityError;
      double totalError;

      // Getting coefficients and parameters.
      thisMutex.lock();
      try {
        input = this.pidSource.getAsDouble();

        P = Kp;
        I = Ki;
        D = Kd;

        period = this.period;

        maximumIntegral = this.maximumIntegral;
        minimumIntegral = this.minimumIntegral;

        maximumOutput = this.maximumOutput;
        minimumOutput = this.minimumOutput;

        totalError = this.totalError;
        positionError = getContinuousError(setpoint - input);
        velocityError = (positionError - prevError) / period;

      } finally {
        thisMutex.unlock();
      }

      // Storage for function outputs
      double result;

      // Clamping Integral term
      if (I != 0) {
        totalError =
            MathUtil.clamp(
                totalError + positionError * period, minimumIntegral / I, maximumIntegral / I);
      }

      // Calculating PIDController result
      result =
          calculateProportional(P, positionError)
              + calculateIntegral(I, totalError)
              + calculateDerivative(D, velocityError)
              + calculateFeedForward(this.setpoint);

      // Clamping output between minimum and maximum
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
        this.prevError = positionError;
        this.totalError = totalError;
        prevMeasurement = input;
        this.result = result;
        this.positionError = positionError;
        this.velocityError = velocityError;
      } finally {
        thisMutex.unlock();
      }
    }
  }

  // Separating calculate for each term so controller can be extended
  protected synchronized double calculateProportional(double p, double error) {
    return p * error;
  }

  protected synchronized double calculateIntegral(double i, double totalerror) {
    return i * totalerror;
  }

  protected synchronized double calculateDerivative(double d, double derror) {
    return d * derror;
  }

  protected synchronized double calculateFeedForward(double setpoint) {
    thisMutex.lock();
    try {
      return Kf;
    } finally {
      thisMutex.unlock();
    }
  }

  public Notifier getNotifier() {
    return notifier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.setSafeState(this::reset);
    builder.addDoubleProperty("p", this::getP, this::setP);
    builder.addDoubleProperty("i", this::getI, this::setI);
    builder.addDoubleProperty("d", this::getD, this::setD);
    builder.addDoubleProperty("f", this::getF, this::setF);
    builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    builder.addBooleanProperty("enabled", this::isEnabled, this::setEnabled);
  }

  /**
   * Wraps error around for continuous inputs. The original error is returned if continuous mode is
   * disabled.
   *
   * @param error The current error of the PID controller.
   * @return Error for continuous inputs.
   */
  public double getContinuousError(double error) {
    thisMutex.lock();
    try {
      if (continuous && inputRange > 0) {
        error %= inputRange;
        if (Math.abs(error) > inputRange / 2) {
          if (error > 0) {
            return error - inputRange;
          } else {
            return error + inputRange;
          }
        }
      }
      return error;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Sets the minimum and maximum values expected from the input.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void setInputRange(double minimumInput, double maximumInput) {
    this.thisMutex.lock();
    try {
      this.minimumInput = minimumInput;
      this.maximumInput = maximumInput;
      inputRange = maximumInput - minimumInput;

      // Clamp setpoint to new input
      if (maximumInput > minimumInput) {
        setpoint = MathUtil.clamp(setpoint, minimumInput, maximumInput);
      }

    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Return the last sampled measurement of the controller.
   *
   * <p>Note:Mainly used for calculating acceleration based feedforward, not the controller output.
   * See {@link this#get()}.
   *
   * @return the latest calculated output
   */
  public double getMeasurement() {
    thisMutex.lock();
    try {
      return prevMeasurement;
    } finally {
      thisMutex.unlock();
    }
  }

  /**
   * Return the current PID result This is always centered on zero and constrained the the max and
   * min outs.
   *
   * @return the latest calculated output
   */
  public double get() {
    thisMutex.lock();
    try {
      return result;
    } finally {
      thisMutex.unlock();
    }
  }

  /** Resetting PIDController values. */
  public void reset() {
    disable();

    thisMutex.lock();
    try {
      prevError = 0;
      totalError = 0;
      result = 0;
      setpoint = 0;
    } finally {
      thisMutex.unlock();
    }
  }

  /** Begin running the PIDController. */
  public void enable() {
    thisMutex.lock();
    try {
      enabled = true;
    } finally {
      thisMutex.unlock();
    }
  }

  /** Stop running the PIDController, this sets the output to zero before stopping. */
  public void disable() {
    // Ensures enabled check and pidWrite() call occur atomically
    pidOutputMutex.lock();
    try {
      thisMutex.lock();
      try {
        enabled = false;
      } finally {
        thisMutex.unlock();
      }

      pidOutput.accept(0);
    } finally {
      pidOutputMutex.unlock();
    }
  }
  /** Set the enabled state of the PIDController. */
  public void setEnabled(boolean enable) {
    if (enable) {
      enable();
    } else {
      disable();
    }
  }

  /** Return true if PIDController is enabled. */
  public boolean isEnabled() {
    thisMutex.lock();
    try {
      return enabled;
    } finally {
      thisMutex.unlock();
    }
  }
}
