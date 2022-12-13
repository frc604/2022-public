// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robot2022.commands;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Intake;
import com._604robotics.robot2022.subsystems.Launcher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PurgeFromLauncher extends CommandBase {
    private BallManager ballManager;
    private Intake intake;
    private Launcher launcher;
    private BallPath ballPath;

    private boolean withDistance;


    /** Creates a new RunIntake. */
    public PurgeFromLauncher(BallManager ballManager, Intake intake, Launcher launcher, BallPath ballPath, boolean withDistance) {
        addRequirements(ballManager, launcher, ballPath);

        this.ballManager = ballManager;
        this.intake = intake;
        this.launcher = launcher;
        this.ballPath = ballPath;

        this.withDistance = withDistance;
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
        if (withDistance) {
            launcher.launch();

            if (launcher.atSpeed()) {
                ballPath.launchBall();
            }
        } else {
            launcher.launchFromDistance(Units.inchesToMeters(120.0));

            if (launcher.atSpeedWithoutDistance()) {
                ballPath.launchBall();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ballManager.handleIntakeDeploymentAtCommandEnd(intake);
        ballPath.emptyTopQueue();
        ballPath.emptyBottomQueue();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
