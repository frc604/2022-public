package frc.quixlib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This swerve drive controller can be used to follow trajectories using a swerve drivetrain.
 *
 * <p>The swerve drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward.
 */
public class QuixSwerveController {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_thetaController;

  /**
   * Constructs a swerve drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A PID controller to respond to error in angle.
   */
  public QuixSwerveController(
      PIDController xController, PIDController yController, PIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the swerve drive controller.
   *
   * @param currentPose The current pose.
   * @param targetPose The desired pose.
   * @param xVelocityRef The field-x velocity reference. Used for feedforward.
   * @param yVelocityRef The field-y velocity reference. Used for feedforward.
   * @param thetaVelocityRef The field-theta velocity reference. Used for feedforward.
   * @return The next output of the swerve drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d targetPose,
      double xVelocityRef,
      double yVelocityRef,
      double thetaVelocityRef) {
    m_poseError = targetPose.relativeTo(currentPose);

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xVelocityRef, yVelocityRef, thetaVelocityRef, currentPose.getRotation());
    }

    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaFeedback =
        m_thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xVelocityRef + xFeedback,
        yVelocityRef + yFeedback,
        thetaVelocityRef + thetaFeedback,
        currentPose.getRotation());
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }
}
