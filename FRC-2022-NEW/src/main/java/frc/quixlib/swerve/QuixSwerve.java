package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.devices.QuixIMU;
import frc.quixlib.localization.QuixSwerveLocalizer;
import frc.quixlib.localization.SwerveDriveOdometryMeasurement;
import frc.quixlib.math.MathUtils;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.QuixVisionCamera;

public abstract class QuixSwerve extends SubsystemBase {
  private final QuixIMU m_imu;
  private final QuixVisionCamera m_camera;
  private final QuixSwerveModule[] m_modules;
  private final SwerveDriveKinematics m_kinematics;
  private final QuixSwerveModuleSetpointGenerator m_setpointGenerator;
  private final QuixSwerveLocalizer m_localizer;
  private final double m_maxDriveSpeed;
  private final QuixSwerveController m_driveController;

  private final Field2d m_fieldViz;
  private final Mechanism2d m_viz = new Mechanism2d(100, 100);
  private final MechanismRoot2d[] m_vizModuleRoots;
  private final MechanismLigament2d[] m_vizModuleCurrentState;
  private final MechanismLigament2d[] m_vizModuleTargetState;

  /**
   * Swerve drive class that handles all swerve-related functions, including kinematics, control,
   * and localization.
   */
  public QuixSwerve(
      QuixIMU imu,
      QuixVisionCamera camera,
      double maxDriveSpeed,
      double maxModuleAcceleration,
      double maxModuleSteeringRate,
      QuixSwerveController driveController,
      Fiducial[] targets,
      Field2d fieldViz) {
    m_imu = imu;
    m_camera = camera;
    m_modules = createModules();
    m_kinematics = new SwerveDriveKinematics(getModulePositions());
    m_setpointGenerator =
        new QuixSwerveModuleSetpointGenerator(
            m_kinematics,
            getModuleStates(),
            maxDriveSpeed,
            maxModuleAcceleration,
            maxModuleSteeringRate);
    m_localizer =
        new QuixSwerveLocalizer(
            m_kinematics,
            new Rotation2d(m_imu.getContinuousYaw()),
            getModulePositionStates(),
            new Pose2d(),
            targets);
    m_maxDriveSpeed = maxDriveSpeed;
    m_driveController = driveController;
    zeroModuleEncoders();

    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Setup Viz
    m_fieldViz = fieldViz;
    SmartDashboard.putData("Swerve Viz", m_viz);
    m_vizModuleRoots = new MechanismRoot2d[m_modules.length];
    m_vizModuleCurrentState = new MechanismLigament2d[m_modules.length];
    m_vizModuleTargetState = new MechanismLigament2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      m_vizModuleRoots[i] =
          m_viz.getRoot(
              "Swerve Module ID: " + i,
              50 + m_modules[i].getPosition().getX() * 100,
              50 + m_modules[i].getPosition().getY() * 100);
      m_vizModuleCurrentState[i] =
          m_vizModuleRoots[i].append(
              new MechanismLigament2d("Current State", 0, 0, 10, new Color8Bit(Color.kRed)));
      m_vizModuleTargetState[i] =
          m_vizModuleRoots[i].append(
              new MechanismLigament2d("Target State", 0, 0, 5, new Color8Bit(Color.kGreen)));
    }
  }

  /**
   * Returns an array of QuixSwerveModules that define the individual module configurations. Called
   * on construction.
   */
  protected abstract QuixSwerveModule[] createModules();

  /** Simulate vision targets based on the simulated pose. Called within simulationPeriodic(). */
  protected abstract void simulateVision(Pose2d simPose);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update localization only when enabled.
    if (DriverStation.isEnabled()) {
      var odometryMeasurement =
          new SwerveDriveOdometryMeasurement(
              new Rotation2d(m_imu.getContinuousYaw()), getModulePositionStates());
      var visionMeasurement = m_camera.getLatestMeasurement();
      if (visionMeasurement.hasTargets()) {
        m_localizer.update(odometryMeasurement, visionMeasurement);
      } else {
        m_localizer.update(odometryMeasurement, null);
      }
    }

    // Plot
    updateModuleViz();
    m_fieldViz
        .getObject("Odometry")
        .setPose(MathUtils.convertPoseToCenterOfField(m_localizer.getOdometryPose()));
    m_fieldViz
        .getObject("Localizer Raw")
        .setPose(MathUtils.convertPoseToCenterOfField(m_localizer.getRawPose()));
    m_fieldViz
        .getObject("Localizer")
        .setPose(MathUtils.convertPoseToCenterOfField(m_localizer.getPose()));
  }

  /** Sets the IMU and localizer to the given pose. */
  public void resetPose(Pose2d pose) {
    m_imu.setContinuousYaw(pose.getRotation().getRadians());
    m_localizer.resetPose(
        new Rotation2d(m_imu.getContinuousYaw()), getModulePositionStates(), pose);
  }

  /**
   * Drive with open loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveOpenLoop(
      double xVel, double yVel, double thetaVel, boolean fieldRelative, double allowedScrub) {
    drive(xVel, yVel, thetaVel, fieldRelative, allowedScrub, false);
  }

  /**
   * Drive with closed loop module velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveClosedLoop(
      double xVel, double yVel, double thetaVel, boolean fieldRelative, double allowedScrub) {
    drive(xVel, yVel, thetaVel, fieldRelative, allowedScrub, true);
  }

  /**
   * Drive with the given translation and rotation velocities.
   *
   * @param xVel Translation velocity in m/s (X+ forward)
   * @param yVel Translation velocity in m/s (Y+ left)
   * @param thetaVel Rotation velocity in rad/s (CCW+)
   * @param fieldRelative Whether the provided velocities are in field-frame or robot-frame
   * @param allowedScrub Allowable module scrub in m/s
   * @param isClosedLoop Whether to use closed-loop module velocities
   */
  private void drive(
      double xVel,
      double yVel,
      double thetaVel,
      boolean fieldRelative,
      double allowedScrub,
      boolean isClosedLoop) {
    // Hacky lead-compensator to address drift when translating while rotating.
    final double kYawDelay = 0.03; // s
    final ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xVel,
                yVel,
                thetaVel,
                new Rotation2d(
                    m_imu.getContinuousYaw()
                        + getChassisSpeeds().omegaRadiansPerSecond * kYawDelay))
            : new ChassisSpeeds(xVel, yVel, thetaVel);
    setDesiredModuleStates(
        m_setpointGenerator.getFeasibleModuleStates(desiredChassisSpeeds, allowedScrub),
        isClosedLoop);
  }

  /**
   * Drives the swerve to the target pose.
   *
   * @param targetPose The target pose in field-frame
   * @param xVelocityRef Field-relative x-velocity feed-forward
   * @param yVelocityRef Field-relative y-velocity feed-forward
   * @param thetaVelocityRef Theta-velocity feed-forward
   * @param allowedScrub Allowable module scrub in m/s
   */
  public void driveToPose(
      Pose2d targetPose,
      double xVelocityRef,
      double yVelocityRef,
      double thetaVelocityRef,
      double allowedScrub) {
    ChassisSpeeds fieldRelativeChassisSpeeds =
        m_driveController.calculate(
            m_localizer.getPose(), targetPose, xVelocityRef, yVelocityRef, thetaVelocityRef);
    driveOpenLoop(
        fieldRelativeChassisSpeeds.vxMetersPerSecond,
        fieldRelativeChassisSpeeds.vyMetersPerSecond,
        fieldRelativeChassisSpeeds.omegaRadiansPerSecond,
        /*fieldRelative=*/ false,
        allowedScrub);
  }

  /** Drive open loop with zero velocity. */
  public void stop() {
    driveOpenLoop(0.0, 0.0, 0.0, /*fieldRelative=*/ false, Double.POSITIVE_INFINITY);
  }

  private ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  private Translation2d[] getModulePositions() {
    Translation2d[] positions = new Translation2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositionStates() {
    SwerveModulePosition[] states = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getPositionState();
    }
    return states;
  }

  private void zeroModuleEncoders() {
    for (var module : m_modules) {
      module.zeroToAbsPosition();
    }
  }

  private void setDesiredModuleStates(SwerveModuleState[] desiredStates, boolean isClosedLoop) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(desiredStates[i], isClosedLoop);
    }
  }

  private void updateModuleViz() {
    var moduleScrubs = m_setpointGenerator.computeModuleScrubs(getModuleStates());
    double kSpeedVizScalar = 5.0;
    for (int i = 0; i < m_modules.length; i++) {
      var currentState = m_modules[i].getState();
      m_vizModuleCurrentState[i].setAngle(currentState.angle);
      m_vizModuleCurrentState[i].setLength(kSpeedVizScalar * currentState.speedMetersPerSecond);
      m_vizModuleCurrentState[i].setLineWeight(5 + 100 * (moduleScrubs[i] / m_maxDriveSpeed));

      var targetState = m_modules[i].getLastCommandedState();
      m_vizModuleTargetState[i].setAngle(targetState.angle);
      m_vizModuleTargetState[i].setLength(kSpeedVizScalar * targetState.speedMetersPerSecond);

      SmartDashboard.putNumber("Module Scrub " + i, moduleScrubs[i]);
    }
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private Pose2d m_simPose = new Pose2d();

  /** Sets the simulated pose to the given pose. */
  public void resetSimPose(Pose2d pose) {
    m_simPose = pose;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // TODO: Sim jointly as one system.
    for (var module : m_modules) {
      module.updateSimPeriodic();
    }

    // Update pose by integrating ChassisSpeeds.
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    m_simPose =
        m_simPose.transformBy(
            new Transform2d(
                new Translation2d(
                    chassisSpeeds.vxMetersPerSecond * TimedRobot.kDefaultPeriod,
                    chassisSpeeds.vyMetersPerSecond * TimedRobot.kDefaultPeriod),
                new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod)));
    m_fieldViz.setRobotPose(MathUtils.convertPoseToCenterOfField(m_simPose));

    // Update IMU based on sim pose.
    m_imu.setSimContinuousYaw(m_simPose.getRotation().getRadians());

    // Simulate vision based on sim pose.
    simulateVision(m_simPose);
  }
  // --- END STUFF FOR SIMULATION ---
}
