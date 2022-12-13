// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.quixlib.swerve.QuikPlanSwerveTrajectoryReader;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class FollowQuikplanCommand extends CommandBase {
  private final QuikPlanSwerveTrajectoryReader m_reader;
  private final Swerve m_swerve;
  private final Timer m_timer = new Timer();

  public FollowQuikplanCommand(QuikPlanSwerveTrajectoryReader reader, Swerve swerve) {
    m_reader = reader;
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetPose(m_reader.getInitialPose());
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double curTime = m_timer.get();
    final var targetState = m_reader.getState(curTime);
    m_swerve.driveToPose(
        targetState.getPose(),
        targetState.getXVelRef(),
        targetState.getYVelRef(),
        targetState.getThetaVelRef(),
        Constants.Swerve.autoScrubLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_reader.getTotalTime());
  }
}
