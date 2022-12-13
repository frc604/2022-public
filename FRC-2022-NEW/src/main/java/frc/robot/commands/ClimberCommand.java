package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  private final ClimberSubsystem m_subsystem;
  private double m_startTime;

  public ClimberCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - m_startTime;
    if (elapsedTime < 2.0) {
      m_subsystem.climbUp();
    } else if (elapsedTime < 4.0) {
      m_subsystem.climbDown();
    } else {
      m_subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsedTime = Timer.getFPGATimestamp() - m_startTime;
    return elapsedTime > 4.0;
  }
}
