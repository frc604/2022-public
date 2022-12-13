// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.balls.BallManager.BallPathState;
import com._604robotics.robot2022.balls.BallManager.BallState;
import com._604robotics.robot2022.commands.EjectFromIntake;
import com._604robotics.robot2022.commands.EjectFromLauncher;
import com._604robotics.robot2022.commands.MoveToTopQueue;
import com._604robotics.robot2022.commands.PurgeFromLauncher;
import com._604robotics.robot2022.commands.RunIntake;
import com._604robotics.robot2022.commands.RunIntakeAndMoveToTopQueue;
import com._604robotics.robot2022.commands.ShootFromLauncher;
import com._604robotics.robot2022.commands.SpinUpLauncher;
import com._604robotics.robot2022.commands.TeleopSwerve;
import com._604robotics.robot2022.commands.TeleopSwerveAim;
import com._604robotics.robot2022.commands.TeleopSwerveAimRadial;
import com._604robotics.robot2022.commands.YeetFromIntake;
import com._604robotics.robot2022.commands.auto.QuixSwerveControllerCommand;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Climber;
import com._604robotics.robot2022.subsystems.Intake;
import com._604robotics.robot2022.subsystems.LEDs;
import com._604robotics.robot2022.subsystems.Launcher;
import com._604robotics.robot2022.subsystems.Swerve;
import com._604robotics.robotnik.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.auto.QuikPlanSwerveReader.TrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  /* Controllers */
  public final XboxController driver = new XboxController(0);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final Trigger leftTrigger = new Trigger(() -> driver.getLeftTriggerAxis() > 0.2);
  private final Trigger advanceClimb = new Trigger(() -> driver.getStartButton() && driver.getBackButton());
  private final Trigger resetClimb = new Trigger(() -> driver.getXButton() && driver.getAButton());

  // Spin up launcher when launching, auto aiming, or SOTF.
  private final Trigger spinUpLauncher = new Trigger(() -> driver.getBButton() || driver.getLeftBumper() || driver.getLeftTriggerAxis() > 0.2);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final ShotCalculator shotCalculator = new ShotCalculator(swerve);
  private final Intake intake = new Intake();
  private final BallPath ballPath = new BallPath();
  private final Launcher launcher = new Launcher(shotCalculator);
  private final Climber climber = new Climber(swerve::getRobotYTilt, swerve::getRobotYTiltRate);
  private final LEDs leds = new LEDs(
    spinUpLauncher,
    ballPath::redCount,
    ballPath::blueCount,
    launcher::atSpeed
  );

  /* Ball Manager */
  private final BallManager ballManager = new BallManager(
      ballPath::getTopQueue,
      ballPath::getBottomQueue,
      ballPath::hasIncorrectBallInTriangle,
      () -> driver.getRightTriggerAxis() > 0.2,
      driver::getBButton,
      () -> driver.getPOV() != -1
  );

  /* Ball Path Triggers */
  private boolean shouldYeet = false;

  private final Trigger runIntake = new Trigger(ballManager::shouldRunIntake);
  private final Trigger runIntakeAndMoveToTopQueue = new Trigger(ballManager::shouldRunIntakeAndMoveToTopQueue);
  private final Trigger moveToTopQueue = new Trigger(ballManager::shouldMoveToTopQueue);
  private final Trigger ejectFromIntake = new Trigger(ballManager::shouldEjectFromIntake);
  private final Trigger ejectFromLauncher = new Trigger(ballManager::shouldEjectFromLauncher);
  private final Trigger shootFromLauncher = new Trigger(ballManager::shouldShootFromLauncher);
  private final Trigger purgeFromLauncher = new Trigger(ballManager::shouldPurgeFromLauncher);
  private final Trigger purgeFromLauncherWithDistance = new Trigger(ballManager::shouldPurgeFromLauncherWithDistance);
  private final Trigger yeet = new Trigger(() -> shouldYeet);

  /* Auto */
  private final QuikPlanSwerveReader reader = new QuikPlanSwerveReader();

  public RobotContainer() {
    /* Default Commands */
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, true, true));

    intake.setDefaultCommand(new PerpetualCommand(new InstantCommand(intake::stop, intake)));
    ballPath.setDefaultCommand(new PerpetualCommand(new InstantCommand(ballPath::stop, ballPath)));
    launcher.setDefaultCommand(new PerpetualCommand(new InstantCommand(launcher::stop, launcher)));

    /* Ball Path Command Bindings */
    runIntake.whileActiveOnce(new RunIntake(ballManager, intake, ballPath));
    runIntakeAndMoveToTopQueue.whileActiveOnce(new RunIntakeAndMoveToTopQueue(ballManager, intake, ballPath));
    moveToTopQueue.whileActiveOnce(new MoveToTopQueue(ballManager, intake, ballPath));
    ejectFromIntake.whileActiveOnce(new EjectFromIntake(ballManager, intake, ballPath));
    ejectFromLauncher.whileActiveOnce(new EjectFromLauncher(ballManager, intake, launcher, ballPath));
    spinUpLauncher.whileActiveOnce(new SpinUpLauncher(launcher));
    shootFromLauncher.whileActiveOnce(new ShootFromLauncher(ballManager, intake, ballPath, launcher::atSpeed));
    purgeFromLauncher.whileActiveOnce(new PurgeFromLauncher(ballManager, intake, launcher, ballPath, false));
    purgeFromLauncherWithDistance.whileActiveOnce(new PurgeFromLauncher(ballManager, intake, launcher, ballPath, true));
    yeet.whileActiveOnce(new YeetFromIntake(ballManager, ballPath, intake));

    ballPath.resetTopBallpathEncoder();
    ballPath.resetBottomBallpathEncoder();

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenHeld(new InstantCommand(() -> swerve.zeroContinousYaw(), swerve));
    rightBumper.whileActiveOnce(new TeleopSwerve(swerve, driver, false, true));
    leftBumper.whileHeld(new TeleopSwerveAim(swerve, driver, true, true));
    leftTrigger.whileActiveOnce(new TeleopSwerveAimRadial(swerve, driver, shotCalculator, true, true));

    advanceClimb.whileActiveOnce(new InstantCommand(climber::advanceCurrentState));
    resetClimb.whileActiveOnce(new InstantCommand(climber::resetClimbState));
  }

  public void resetBallPath() {
    ballPath.resetTopBallpathEncoder();
    ballPath.resetBottomBallpathEncoder();
  }

  private void setYeet(boolean shouldYeet) {
    this.shouldYeet = shouldYeet;
  }

  public void swerveTeleopPeriodic() {
    swerve.quixsamPeriodic();
  }

  public Command getAutonomousCommand() {
    reader.clearLoadedData();
    reader.loadChosenFile();

    var initialState = reader.getState(0.0);

    Pose2d initialPose = new Pose2d(
      initialState.get(TrajectoryState.x.index),
      initialState.get(TrajectoryState.y.index),
      new Rotation2d(initialState.get(TrajectoryState.Theta.index))
    );

    ballPath.setBottomQueue(BallState.CORRECT);
    ballPath.setTopQueue(BallState.CORRECT);
    ballManager.setCurrentState(BallPathState.FULL_FULL);

    swerve.zeroContinousYaw(Units.radiansToDegrees(initialState.get(TrajectoryState.Theta.index)));
    swerve.resetLocalizerPose(initialPose);

    return new QuixSwerveControllerCommand(
      reader,
      ballManager,
      this::setYeet,
      swerve,
      swerve::getEstimatedPose,
      swerve::getContinousYaw,
      swerve.getKinematics(),
      Calibration.Auto.controller,
      swerve::setModuleStates
    );
  }
}
