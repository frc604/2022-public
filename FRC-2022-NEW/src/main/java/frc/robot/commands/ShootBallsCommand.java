// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallpathSubsystem;
import frc.robot.subsystems.Launcher;

public class ShootBallsCommand extends CommandBase {
  private final Launcher m_launcher;
  private final BallpathSubsystem m_ballpath;

  public ShootBallsCommand(Launcher launcher, BallpathSubsystem ballpath) {
    m_launcher = launcher;
    m_ballpath = ballpath;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcher);
    addRequirements(ballpath);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.spinRoller();
    m_ballpath.sendToLauncher();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.stopRoller();
    m_ballpath.stopSendToLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
