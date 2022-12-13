package com._604robotics.robot2022.balls;

import java.util.function.Supplier;

import com._604robotics.robot2022.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallManager extends SubsystemBase {
    private final Supplier<BallState> topQueue;
    private final Supplier<BallState> bottomQueue;
    private final Supplier<Boolean> hasIncorrectBallInTriangle;
    private final Supplier<Boolean> intakeButtonState;
    private final Supplier<Boolean> shootButtonState;
    private final Supplier<Boolean> purgeButtonState;
    
    private Action currentAction = Action.NONE;
    private BallPathState currentState = BallPathState.EMPTY_EMPTY;
    private boolean actionFinished = false;

    private boolean intakeLatch = false;

    private boolean intakeAuto = false;
    private boolean shootAuto = false;

    public BallManager(
        Supplier<BallState> topQueue,
        Supplier<BallState> bottomQueue,
        Supplier<Boolean> hasIncorrectBallInTriangle,
        Supplier<Boolean> intakeButtonState,
        Supplier<Boolean> shootButtonState,
        Supplier<Boolean> purgeButtonState) {
        this.topQueue = topQueue;
        this.bottomQueue = bottomQueue;
        this.hasIncorrectBallInTriangle = hasIncorrectBallInTriangle;
        this.intakeButtonState = intakeButtonState;
        this.shootButtonState = shootButtonState;
        this.purgeButtonState = purgeButtonState;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Current Action", currentAction.toString());
        SmartDashboard.putBoolean("Action Finished", actionFinished);
        SmartDashboard.putString("Current State", currentState.toString());

        // Handle State Transisitions
        switch (currentState) {
            case EMPTY_EMPTY:
                if (actionFinished && currentAction == Action.RUN_INTAKE) {
                    actionFinished = false;
                    currentState = BallPathState.FULL_EMPTY;
                }
                break;

            case EMPTY_FULL:                
                if (actionFinished && currentAction == Action.SEND_TO_LAUNCHER) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_EMPTY;
                } else if (actionFinished && currentAction == Action.RUN_INTAKE) {
                    actionFinished = false;
                    currentState = BallPathState.FULL_FULL;
                } else if (currentAction == Action.RUN_INTAKE && hasIncorrectBallInTriangle.get()) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_FULL_EJECTING;
                }
                break;

            case EMPTY_FULL_EJECTING:
                if (actionFinished && currentAction == Action.EJECT_FROM_INTAKE) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_FULL;
                }
                break;

            case FULL_EMPTY:
                if (actionFinished && currentAction == Action.EJECT_FROM_INTAKE) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_EMPTY;
                } else if (actionFinished &&
                           (currentAction == Action.RUN_INTAKE_AND_MOVE_TO_TOP_QUEUE ||
                            currentAction == Action.MOVE_TO_TOP_QUEUE)) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_FULL;
                }
                break;

            case FULL_FULL:
                if (actionFinished && currentAction == Action.EJECT_FROM_INTAKE) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_FULL;
                } else if (actionFinished && currentAction == Action.SEND_TO_LAUNCHER) {
                    actionFinished = false;
                    currentState = BallPathState.EMPTY_EMPTY;
                }
                break;
        }

        // Schedule Actions
        switch(currentState) {
            case EMPTY_EMPTY:
                if (shouldIntakeRun()) {
                    currentAction = Action.RUN_INTAKE;
                } else {
                    currentAction = Action.NONE;
                }
                break;

            case EMPTY_FULL:
                if (shouldLauncherRun()) {
                }

                if (shouldIntakeRun()) {
                    currentAction = Action.RUN_INTAKE;
                } else if (shouldLauncherRun() && topQueue.get() == BallState.CORRECT) {
                    currentAction = Action.SEND_TO_LAUNCHER;
                } else {
                    currentAction = Action.NONE;
                }
                break;

            case EMPTY_FULL_EJECTING:
                currentAction = Action.EJECT_FROM_INTAKE;
                break;

            case FULL_EMPTY:
                if (shouldIntakeRun()) {
                    currentAction = Action.RUN_INTAKE_AND_MOVE_TO_TOP_QUEUE;
                } else {
                    currentAction = Action.MOVE_TO_TOP_QUEUE;
                }
                break;

            case FULL_FULL:
                if (shouldIntakeRun() && bottomQueue.get() == BallState.INCORRECT) {
                    currentAction = Action.EJECT_FROM_INTAKE;
                } else if (shouldIntakeRun() && topQueue.get() == BallState.INCORRECT) {
                    currentAction = Action.SEND_TO_LAUNCHER;
                } else if (shouldLauncherRun() && bottomQueue.get() == BallState.CORRECT && topQueue.get() == BallState.CORRECT) {
                    currentAction = Action.SEND_TO_LAUNCHER;
                } else {
                    currentAction = Action.NONE;
                }
                break;
        }

        if (shouldPurge()) {
            currentAction = Action.PURGE;
            currentState = BallPathState.EMPTY_EMPTY;
            actionFinished = false;
        }
    }

    // This needs to be called at the end of every ballpath-related command.
    public void handleIntakeDeploymentAtCommandEnd(Intake intake) {
        // Keep intake deployed in auto.
        if (this.shootAuto || this.intakeAuto) {
            intake.deploy();
            return;
        }

        // If the intake button is not pressed, always retract.
        if (!intakeButtonState.get()) {
            intake.retract();
            return;
        }

        // Otherwise, decide based on whether we have space for another correct ball.
        boolean hasSpaceForCorrectBall = bottomQueue.get() != BallState.CORRECT || topQueue.get() != BallState.CORRECT;
        if (hasSpaceForCorrectBall) {
            intake.deploy();
        } else {
            intake.retract();
        }
    }

    private boolean shouldIntakeRun() {
        return intakeButtonState.get() || intakeAuto;
    }

    private boolean shouldLauncherRun() {
        return shootButtonState.get();
    }

    private boolean shouldPurge() {
        return purgeButtonState.get() || shootAuto;
    }

    public void forceIntakeLatch(boolean value) {
        intakeLatch = value;
    }

    public void setCurrentState(BallPathState currentState) {
        this.currentState = currentState;
    }

    public void setIntakeAuto(boolean intakeAuto) {
        this.intakeAuto = intakeAuto;
    }

    public void setShootAuto(boolean shootAuto) {
        this.shootAuto = shootAuto;
    }

    public void notifyActionFinished() {
        actionFinished = true;
    }

    public boolean shouldRunIntake() {
        return currentAction == Action.RUN_INTAKE;
    }

    public boolean shouldRunIntakeAndMoveToTopQueue() {
        return currentAction == Action.RUN_INTAKE_AND_MOVE_TO_TOP_QUEUE;
    }

    public boolean shouldMoveToTopQueue() {
        return currentAction == Action.MOVE_TO_TOP_QUEUE;
    }

    public boolean shouldEjectFromIntake() {
        return currentAction == Action.EJECT_FROM_INTAKE;
    }

    public boolean shouldEjectFromLauncher() {
        return currentAction == Action.SEND_TO_LAUNCHER && topQueue.get() == BallState.INCORRECT;
    }

    public boolean shouldShootFromLauncher() {
        return currentAction == Action.SEND_TO_LAUNCHER && topQueue.get() == BallState.CORRECT;
    }
    
    public boolean shouldPurgeFromLauncher() {
        return (currentAction == Action.PURGE) && !this.shootAuto;
    }

    public boolean shouldPurgeFromLauncherWithDistance() {
        return (currentAction == Action.PURGE) && this.shootAuto;
    }

    public BallPathState getCurrentState() {
        return currentState;
    }

    public enum Action {
        NONE,
        RUN_INTAKE,
        RUN_INTAKE_AND_MOVE_TO_TOP_QUEUE,
        MOVE_TO_TOP_QUEUE,
        EJECT_FROM_INTAKE,
        SEND_TO_LAUNCHER,
        PURGE,
    }


    public enum ButtonState {
        PRESSED,
        RELEASED,
    }
    
    public enum BallPathState {
        EMPTY_EMPTY,
        EMPTY_FULL,
        EMPTY_FULL_EJECTING,
        FULL_EMPTY,
        FULL_FULL,
    }

    public enum BallState {
        NONE,
        CORRECT,
        INCORRECT,
    }
}
