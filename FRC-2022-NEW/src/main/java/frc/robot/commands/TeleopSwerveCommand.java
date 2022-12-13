// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.quixlib.swerve.QuixSwerveTeleopControl;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerveCommand extends CommandBase {
  private final Swerve m_swerve;
  private final boolean m_fieldRelative;
  private final XboxController m_xboxController;
  private final QuixSwerveTeleopControl m_teleopControl;

  public TeleopSwerveCommand(
      Swerve swerve, XboxController xboxController, boolean fieldRelative, boolean squared) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    m_swerve = swerve;
    m_xboxController = xboxController;
    m_fieldRelative = fieldRelative;
    m_teleopControl =
        new QuixSwerveTeleopControl(
            Constants.Swerve.linearSlewRate,
            Constants.Swerve.angularSlewRate,
            Constants.Swerve.stickDeadband,
            Constants.Swerve.maxDriveSpeed,
            Constants.Swerve.maxAngularVelocity,
            squared);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Map joystick axes to velocities.
    // Note that translation axes are swapped and negated.
    // Rotation axis is negated.
    final var driveVelocities =
        m_teleopControl.getDriveVelocitiesFromJoysticks(
            -m_xboxController.getLeftY(),
            -m_xboxController.getLeftX(),
            -m_xboxController.getRightX());
    m_swerve.driveOpenLoop(
        driveVelocities.xVel(),
        driveVelocities.yVel(),
        driveVelocities.thetaVel(),
        m_fieldRelative,
        Constants.Swerve.teleopScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
