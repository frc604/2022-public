// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.quixlib.devices.QuixPigeon;
import frc.quixlib.swerve.QuikPlanSwerveTrajectoryReader;
import frc.quixlib.vision.QuixSimCamera;
import frc.quixlib.vision.QuixVisionCamera;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.FollowQuikplanCommand;
import frc.robot.commands.IntakeBallsCommand;
import frc.robot.commands.ShootBallsCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.BallpathSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // Controllers
  public final XboxController driverXbox = new XboxController(0);

  // Driver Buttons
  private final Trigger rightTrigger = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.2);
  private final Trigger advanceClimb =
      new Trigger(() -> driverXbox.getStartButton() && driverXbox.getBackButton());
  private final JoystickButton buttonB =
      new JoystickButton(driverXbox, XboxController.Button.kB.value);

  // Simulation Viz
  private final Field2d fieldViz = new Field2d();
  private final Mechanism2d simViz = new Mechanism2d(100, 100);

  // Sensors
  private final QuixPigeon imu = new QuixPigeon(Constants.pigeonID);
  private final QuixVisionCamera camera = new QuixSimCamera();

  // Subsystems
  private final IntakeSubsystem IntakeSubsystem = new IntakeSubsystem(simViz);
  private final Launcher launcher = new Launcher(simViz);
  private final BallpathSubsystem BallpathSubsystem = new BallpathSubsystem(simViz);
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(simViz);
  private final Swerve swerve = new Swerve(imu, camera, fieldViz);

  // Misc.
  private final QuikPlanSwerveTrajectoryReader trajectoryReader =
      new QuikPlanSwerveTrajectoryReader(fieldViz);

  public RobotContainer() {
    // Default commands
    swerve.setDefaultCommand(new TeleopSwerveCommand(swerve, driverXbox, true, true));

    configureButtonBindings();

    // Visualize in SmartDashboard.
    SmartDashboard.putData("Field", fieldViz);
    SmartDashboard.putData("Robot Viz", simViz);
  }

  private void configureButtonBindings() {
    buttonB.whileTrue(new ShootBallsCommand(launcher, BallpathSubsystem));
    rightTrigger.whileTrue(new IntakeBallsCommand(IntakeSubsystem, BallpathSubsystem));
    advanceClimb.onTrue(new ClimberCommand(climberSubsystem));
  }

  public Command getAutonomousCommand() {
    return new FollowQuikplanCommand(trajectoryReader, swerve);
  }

  /** Runs when the robot is disabled. */
  public void disabledPeriodic() {
    trajectoryReader.loadSelectedFile();
    if (Robot.isSimulation()) {
      swerve.resetSimPose(trajectoryReader.getInitialPose());
    }
  }
}
