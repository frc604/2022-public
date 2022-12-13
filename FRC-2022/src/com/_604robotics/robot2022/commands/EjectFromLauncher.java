// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Intake;
import com._604robotics.robot2022.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class EjectFromLauncher extends CommandBase {
    private BallManager ballManager;
    private Intake intake;
    private Launcher launcher;
    private BallPath ballPath;

    private boolean topBeamBreakTriggered = false;

    /** Creates a new RunIntake. */
    public EjectFromLauncher(BallManager ballManager, Intake intake, Launcher launcher, BallPath ballPath) {
        addRequirements(ballManager, launcher, ballPath);

        this.ballManager = ballManager;
        this.intake = intake;
        this.launcher = launcher;
        this.ballPath = ballPath;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        topBeamBreakTriggered = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        launcher.eject();

        if (launcher.atSpeed()) {
            ballPath.launchBall();
            if (ballPath.getTopBeamBreakTriggered()) {
                topBeamBreakTriggered = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ballManager.handleIntakeDeploymentAtCommandEnd(intake);
        if (!interrupted){
            ballPath.resetTopBallpathEncoder();
            ballPath.resetBottomBallpathEncoder();

            ballPath.emptyTopQueue();

            ballManager.notifyActionFinished();
        } else {
            ballPath.resetTopBallpathEncoderWithCurrentPosition();
            ballPath.resetBottomBallpathEncoderWithCurrentPosition();
        }
        launcher.run(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return topBeamBreakTriggered && !ballPath.getTopBeamBreakTriggered();
    }
}
