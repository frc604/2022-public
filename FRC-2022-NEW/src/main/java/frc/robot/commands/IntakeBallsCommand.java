// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallpathSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBallsCommand extends CommandBase {
  private final IntakeSubsystem m_intakeSubsystem;
  private final BallpathSubsystem m_ballpathSubsystem;

  public IntakeBallsCommand(IntakeSubsystem intakeSubsystem, BallpathSubsystem ballpathSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_ballpathSubsystem = ballpathSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    addRequirements(ballpathSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.deployMethod();
    m_intakeSubsystem.startIntakeSpin();
    m_ballpathSubsystem.recieveFromIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.retractMethod();
    m_intakeSubsystem.stopIntakeSpin();
    m_ballpathSubsystem.stopSendToLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
