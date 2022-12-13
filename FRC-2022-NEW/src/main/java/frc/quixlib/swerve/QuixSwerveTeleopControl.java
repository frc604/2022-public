package frc.quixlib.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.quixlib.math.MathUtils;

public class QuixSwerveTeleopControl {
  private final SlewRateLimiter m_xVelRateLimiter;
  private final SlewRateLimiter m_yVelRateLimiter;
  private final SlewRateLimiter m_angularRateLimiter;
  private final double m_stickDeadband;
  private final double m_maxDriveSpeed;
  private final double m_maxAngularVelocity;
  private final boolean m_squared;
  private final DriveVelocities m_driveVelocities = new DriveVelocities(0.0, 0.0, 0.0);

  /**
   * Helper to compute drive velocities from joystick values.
   *
   * @param linearSlewRate m/s/s
   * @param angularSlewRate rad/s/s
   * @param stickDeadband joystick deadband as a percentage (0 to 1.0)
   * @param maxDriveSpeed m/s/s
   * @param maxAngularVelocity rad/s/s
   * @param squared whether or not to square the deadbanded joystick
   */
  public QuixSwerveTeleopControl(
      double linearSlewRate,
      double angularSlewRate,
      double stickDeadband,
      double maxDriveSpeed,
      double maxAngularVelocity,
      boolean squared) {
    m_xVelRateLimiter = new SlewRateLimiter(linearSlewRate);
    m_yVelRateLimiter = new SlewRateLimiter(linearSlewRate);
    m_angularRateLimiter = new SlewRateLimiter(angularSlewRate);
    m_stickDeadband = stickDeadband;
    m_maxDriveSpeed = maxDriveSpeed;
    m_maxAngularVelocity = maxAngularVelocity;
    m_squared = squared;
  }

  public class DriveVelocities {
    private double m_xVel;
    private double m_yVel;
    private double m_thetaVel;

    public DriveVelocities(double xVel, double yVel, double thetaVel) {
      m_xVel = xVel;
      m_yVel = yVel;
      m_thetaVel = thetaVel;
    }

    /** X velocity in m/s. */
    public double xVel() {
      return m_xVel;
    }

    /** Y velocity in m/s. */
    public double yVel() {
      return m_yVel;
    }

    /** Rotational velocity in m/s. */
    public double thetaVel() {
      return m_thetaVel;
    }
  }

  /**
   * Compute the translation and rotation velocities given joystick axis inputs.
   *
   * @param xAxis Joystick axis that corresponds to the forward axis of the robot.
   * @param yAxis Joystick axis that corresponds to the left axis of the robot.
   * @param rAxis Joystick axis that corresponds to the CCW rotation of the robot.
   */
  public DriveVelocities getDriveVelocitiesFromJoysticks(double xAxis, double yAxis, double rAxis) {

    /* Deadbands */
    var deadbandedXY = applyDeadbandOnXY(xAxis, yAxis, m_stickDeadband);
    xAxis = deadbandedXY.getFirst();
    yAxis = deadbandedXY.getSecond();
    rAxis = applyDeadband(rAxis, m_stickDeadband);

    if (m_squared) {
      yAxis = square(yAxis);
      xAxis = square(xAxis);
      rAxis = square(rAxis);
    }

    // Convert joystick axes to velocities.
    double xVel = xAxis * m_maxDriveSpeed;
    double yVel = yAxis * m_maxDriveSpeed;
    double angularVel = rAxis * m_maxAngularVelocity;

    // Apply slew rates on velocities.
    m_driveVelocities.m_xVel = m_xVelRateLimiter.calculate(xVel);
    m_driveVelocities.m_yVel = m_yVelRateLimiter.calculate(yVel);
    m_driveVelocities.m_thetaVel = m_angularRateLimiter.calculate(angularVel);
    return m_driveVelocities;
  }

  /** Apply deadband in joystick polar coordinates instead of on the raw joystick values. */
  private static Pair<Double, Double> applyDeadbandOnXY(double x, double y, double deadband) {
    var rAndTheta = MathUtils.cart2pol(x, y);
    double deadbandedR = applyDeadband(rAndTheta.getFirst(), deadband);
    return MathUtils.pol2cart(deadbandedR, rAndTheta.getSecond());
  }

  /**
   * Apply deadband. Ramps linearly from 0 to 1 from (deadband, 1) and 0 to -1 from (-deadband, -1).
   */
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0;
    } else {
      double signedOne = Math.signum(value);
      return (1 / (1 - deadband)) * (value - signedOne) + signedOne;
    }
  }

  private static double square(double value) {
    return Math.signum(value) * value * value;
  }
}
