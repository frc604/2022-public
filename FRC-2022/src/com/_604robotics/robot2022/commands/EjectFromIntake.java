// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EjectFromIntake extends CommandBase {
    private BallManager ballManager;
    private Intake intake;
    private BallPath ballPath;
    private double lastSeenTime;

    /** Creates a new RunIntake. */
    public EjectFromIntake(BallManager ballManager, Intake intake, BallPath ballPath) {
        addRequirements(ballManager, intake, ballPath);

        this.ballManager = ballManager;
        this.intake = intake;
        this.ballPath = ballPath;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.deploy();
        ballPath.resetBottomBallpathEncoder();
        lastSeenTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.eject();
        ballPath.ejectFromIntake();

        if (ballPath.getIntakeBeamBreakTriggered() || ballPath.getBottomBeamBreakTriggered()) {
            lastSeenTime = Timer.getFPGATimestamp();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ballManager.handleIntakeDeploymentAtCommandEnd(intake);

        ballPath.resetTopBallpathEncoder();
        ballPath.resetBottomBallpathEncoder();

        // intake.retract();

        if (!interrupted) {
            ballPath.emptyBottomQueue();
            ballManager.notifyActionFinished();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // End shortly after we last see the ball.
        return Timer.getFPGATimestamp() - lastSeenTime > 0.15;
    }
}
