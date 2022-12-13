// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands;

import java.util.function.Supplier;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Intake;
import com._604robotics.robot2022.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootFromLauncher extends CommandBase {
    private BallManager ballManager;
    private Intake intake;
    private BallPath ballPath;

    private boolean isFinished = false;
    private boolean entryLatch = false;

    Supplier<Boolean> launcherAtSpeed;


    /** Creates a new RunIntake. */
    public ShootFromLauncher(BallManager ballManager, Intake intake, BallPath ballPath, Supplier<Boolean> launcherAtSpeed) {
        addRequirements(ballManager, ballPath);

        this.ballManager = ballManager;
        this.intake = intake;
        this.ballPath = ballPath;
        this.launcherAtSpeed = launcherAtSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        entryLatch = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (launcherAtSpeed.get()) {
            isFinished = ballPath.launchBall();
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
            ballPath.emptyBottomQueue();

            ballManager.notifyActionFinished();
        } else {
            ballPath.resetTopBallpathEncoderWithCurrentPosition();
            ballPath.resetBottomBallpathEncoderWithCurrentPosition();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
