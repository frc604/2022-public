// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands;

import com._604robotics.robot2022.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpinUpLauncher extends CommandBase {
    private Launcher launcher;

    public SpinUpLauncher(Launcher launcher) {
        addRequirements(launcher);
        this.launcher = launcher;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        launcher.launch();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        launcher.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
