// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands.climb;

import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.subsystems.Climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveClimber extends CommandBase {
  private Climber climber;

  private double position;
  private TrapezoidProfile.Constraints trapConstraints;
  private boolean isLoaded;

  /** Creates a new MoveClimber. */
  public MoveClimber(Climber climber, double position, TrapezoidProfile.Constraints trapConstraints, boolean isLoaded) {
    addRequirements(climber);

    this.climber = climber;
    this.position = position;
    this.trapConstraints = trapConstraints;
    this.isLoaded = isLoaded;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setConfig(
      position,
      trapConstraints,
      isLoaded);
    climber.setIsClimbing(true);
    climber.resetClimbTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setIsClimbing(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
