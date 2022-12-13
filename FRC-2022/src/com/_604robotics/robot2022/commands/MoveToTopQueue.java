// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToTopQueue extends CommandBase {
    private BallManager ballManager;
    private Intake intake;
    private BallPath ballPath;

    /** Creates a new RunIntake. */
    public MoveToTopQueue(BallManager ballManager, Intake intake, BallPath ballPath) {
        addRequirements(ballManager, intake, ballPath);

        this.ballManager = ballManager;
        this.intake = intake;
        this.ballPath = ballPath;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ballPath.resetTopBallpathEncoder();
        ballPath.resetBottomBallpathEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ballPath.moveAndKeepBallInTopPosition();

        // System.out.println("AAAAAAAAAAAAA");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ballManager.handleIntakeDeploymentAtCommandEnd(intake);

        ballPath.resetTopBallpathEncoder();
        ballPath.resetBottomBallpathEncoder();

        System.out.println(interrupted);

        if (!interrupted) {
            ballPath.transitionBottomQueueToTopQueue();
            ballPath.moveUpBottomRoller();
            ballManager.notifyActionFinished();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !ballPath.getBottomBeamBreakTriggered();
    }
}
