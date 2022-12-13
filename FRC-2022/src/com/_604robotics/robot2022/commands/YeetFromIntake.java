package com._604robotics.robot2022.commands;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.subsystems.BallPath;
import com._604robotics.robot2022.subsystems.Intake;
import com._604robotics.robot2022.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class YeetFromIntake extends CommandBase {
    private BallManager ballManager;
    private BallPath ballPath;
    private Intake intake;


    /** Creates a new RunIntake. */
    public YeetFromIntake(BallManager ballManager, BallPath ballPath, Intake intake) {
        addRequirements(ballManager, ballPath);

        this.ballManager = ballManager;
        this.ballPath = ballPath;
        this.intake = intake;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.retract();
        ballPath.resetTopBallpathEncoder();
        ballPath.resetBottomBallpathEncoder();        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.yeet();
        ballPath.yeetFromIntake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ballManager.handleIntakeDeploymentAtCommandEnd(intake);

        ballPath.emptyTopQueue();
        ballPath.emptyBottomQueue();

        ballManager.setCurrentState(BallManager.BallPathState.EMPTY_EMPTY);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}