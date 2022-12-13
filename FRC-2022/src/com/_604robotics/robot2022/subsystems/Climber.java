package com._604robotics.robot2022.subsystems;

import java.util.function.DoubleSupplier;

import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.Constants;
import com._604robotics.robot2022.commands.climb.MoveClimber;
import com._604robotics.robotnik.devices.NEOEncoder;
import com._604robotics.robotnik.motorcontrol.Motor;
import com._604robotics.robotnik.motorcontrol.QuixSparkMAX;
import com._604robotics.robotnik.motorcontrol.controllers.SparkPID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private final QuixSparkMAX climberPrimaryMotor = new QuixSparkMAX(
        Constants.Climber.climberMotor0, "Climber Primary Motor", Motor.kNEO, this);
    private final QuixSparkMAX climberFollower0Motor = new QuixSparkMAX(
        Constants.Climber.climberMotor1, "Climber Follower #0 Motor", Motor.kNEO, this);
    private final QuixSparkMAX climberFollower1Motor = new QuixSparkMAX(
        Constants.Climber.climberMotor2, "Climber Follower #1 Motor", Motor.kNEO, this);

    private final NEOEncoder climberEncoder;

    private final SparkPID climberPID;

    private TrapezoidProfile trapProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(0, 0), new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(0, 0)
    );

    private DoubleSupplier robotYTilt;
    private DoubleSupplier robotYTiltRate;
    private double prevTilt = 0.0;  // Used to determine our direction of swing

    private Timer climbTimer = new Timer();
    private boolean isClimbing = false;
    private boolean isLoaded = false;
    private boolean buttonCommandedTransition = false;

    private double filteredPosition = 0.0;
    private double filteredVelocity = 0.0;
    private double targetPosition = 0.0;
    MedianFilter positionFilter = new MedianFilter(5);
    MedianFilter velocityFilter = new MedianFilter(5);
    MedianFilter tiltFilter = new MedianFilter(5);
    MedianFilter TiltRateocityFilter = new MedianFilter(5);
    private int tiltBufferSize = 50;  // 50 samples is 1 second
    CircularBuffer tiltBuffer = new CircularBuffer(tiltBufferSize);

    private ClimberState currentState = ClimberState.REST;

    private final Command firstRungCommand = new MoveClimber(
        this, Calibration.Climber.firstRungPositon, Calibration.Climber.moveFastTrapConstraints, false);

    private final Command touchFirstRungCommand = new MoveClimber(
        this, Calibration.Climber.touchFirstRungPositon, Calibration.Climber.moveFastTrapConstraints, false);

    private final Command partialLiftCommand = new MoveClimber(
        this, Calibration.Climber.partialLiftPosition, Calibration.Climber.moveSlowTrapConstraints, true);
    
    private final Command fullLiftCommand = new MoveClimber(
        this, Calibration.Climber.ratchetPosition, Calibration.Climber.moveTransitionTrapConstraints, true);
    
    private final Command handoffCommand = new MoveClimber(
        this, Calibration.Climber.handoffPosition, Calibration.Climber.moveSlowTrapConstraints, true);

    private final Command reachNextCommand = new MoveClimber(
        this, Calibration.Climber.nextRungPositon, Calibration.Climber.moveFastTrapConstraints, false);
        
    public Climber(DoubleSupplier robotYTilt, DoubleSupplier robotYTiltRate) {
        this.robotYTilt = robotYTilt;
        this.robotYTiltRate = robotYTiltRate;

        climberPrimaryMotor.resetParams();
        climberFollower0Motor.resetParams();
        climberFollower1Motor.resetParams();

        climberEncoder = new NEOEncoder(climberPrimaryMotor, Calibration.Climber.climberRatio);
        climberEncoder.setdistancePerRotation(Calibration.Climber.pitchDiameter * Math.PI);

        climberPID = new SparkPID(climberPrimaryMotor, climberEncoder, Calibration.Climber.climberPID);
        climberPID.setConfig(Calibration.Climber.climberPID);
        // Not sure if the units for iZone are actually correct.
        climberPID.pid.setIZone(Calibration.Climber.iZone, Calibration.Climber.climberPID.getSlot());
        climberPID.pid.setIMaxAccum(Calibration.Climber.iMax, Calibration.Climber.climberPID.getSlot());

        climberPrimaryMotor.setCurrentLimit(10);
        climberFollower0Motor.setCurrentLimit(10);
        climberFollower1Motor.setCurrentLimit(10);

        climberPrimaryMotor.setInverted(Calibration.Climber.invert);
        climberFollower0Motor.follow(climberPrimaryMotor, false);
        climberFollower1Motor.follow(climberPrimaryMotor, false);

        climberPrimaryMotor.controller.setSoftLimit(SoftLimitDirection.kReverse, (float) Calibration.Climber.bottomSoftLimit);
        climberPrimaryMotor.controller.setSoftLimit(SoftLimitDirection.kForward, (float) Calibration.Climber.topSoftLimit);

        climberPrimaryMotor.controller.enableSoftLimit(SoftLimitDirection.kReverse, true);
        climberPrimaryMotor.controller.enableSoftLimit(SoftLimitDirection.kForward, true);

        climberPrimaryMotor.controller.setIdleMode(IdleMode.kBrake);
        climberFollower0Motor.controller.setIdleMode(IdleMode.kBrake);
        climberFollower1Motor.controller.setIdleMode(IdleMode.kBrake);


        climberPrimaryMotor.burnFlashConditionally(true);
        climberFollower0Motor.burnFlashConditionally(true);
        climberFollower1Motor.burnFlashConditionally(true);

        filteredPosition = positionFilter.calculate(climberEncoder.getPosition());
        filteredVelocity = velocityFilter.calculate(climberEncoder.getVelocity());

        resetClimbTimer();
    }

    public void setConfig(double targetPosition, TrapezoidProfile.Constraints constraints, boolean loaded) {
        this.targetPosition = targetPosition;
        System.out.println("SET CONFIG: " + targetPosition + ", " + filteredPosition + ", " + filteredVelocity);
        trapProfile = new TrapezoidProfile(
            constraints,
            new TrapezoidProfile.State(targetPosition, 0.0),
            new TrapezoidProfile.State(filteredPosition, 0.0)
        );
        isLoaded = loaded;
        climbTimer.start();
    }

    public void stop() {
        climberPrimaryMotor.stopMotor();
        this.isClimbing = false;
    }

    public void setIsClimbing(boolean isClimbing) {
        this.isClimbing = isClimbing;
    }

    public void resetClimbTimer() {
        climbTimer.reset();
    }

    public void advanceCurrentState() {
        buttonCommandedTransition = true;
    }

    public void resetClimbState() {
        currentState = ClimberState.REST;
        buttonCommandedTransition = false;
        CommandScheduler.getInstance().schedule(new InstantCommand(() -> {}, this));
    }

    private void transitionState() {
        currentState = ClimberState.values()[currentState.ordinal() + 1];
    }

    private void scheduleIfNotScheduled(Command command) {
        if (this.getCurrentCommand() == null || !this.getCurrentCommand().equals(command)) {
            CommandScheduler.getInstance().schedule(true, command);
        }
    }

    private boolean atTarget() {
        return (Math.abs(filteredPosition - targetPosition) < Calibration.Climber.positionTolerance)
            && trapProfile.isFinished(climbTimer.get());
    }

    @Override
    public void periodic() {
        // System.out.println(atTarget());

        // Compute filtered position and velocity.
        filteredPosition = positionFilter.calculate(climberEncoder.getPosition());
        filteredVelocity = velocityFilter.calculate(climberEncoder.getVelocity());

        // Find the point we swing backwards (in the negative-tilt direction) past the
        // zero point. The transition time is set some amount of time after.
        double curTime = Timer.getFPGATimestamp();
        double curTilt = tiltFilter.calculate(robotYTilt.getAsDouble());
        double curTiltRate = TiltRateocityFilter.calculate(robotYTiltRate.getAsDouble());
        boolean inZeroZone = Math.abs(curTilt - Calibration.Climber.fixedTopZeroAngle) < Calibration.Climber.fixedTopZeroAngleTolerance;
        boolean gyroTransition = false;
        if (inZeroZone && curTiltRate <= 0) {
            System.out.println("TRANSITION!!!!!!!!!!!!!!!!!!!!!!!!! " + curTime + ", " + curTilt + ", " + curTiltRate);
            gyroTransition = true;
        }
        prevTilt = curTilt;

        // Keep track of past tilts.
        tiltBuffer.addFirst(curTilt);

        switch (currentState) {
            case REST: {
                stop();

                if (buttonCommandedTransition) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case REACH_FIRST_RUNG: {
                this.scheduleIfNotScheduled(firstRungCommand);

                if (buttonCommandedTransition && atTarget()) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case TOUCH_FIRST_RUNG: {
                this.scheduleIfNotScheduled(touchFirstRungCommand);

                if (buttonCommandedTransition && atTarget()) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case PARTIAL_LIFT_MID:
            case PARTIAL_LIFT_HIGH: {
                this.scheduleIfNotScheduled(partialLiftCommand);

                // THIS IS SCARY! Gyro controls transition.
                if (buttonCommandedTransition && atTarget() && gyroTransition) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case LEAP_OF_FAITH_TO_HIGH:
            case LEAP_OF_FAITH_TO_TRAVRSAL: {
                this.scheduleIfNotScheduled(fullLiftCommand);

                // Lockout if antenna touch angle isn't settled where we expect it. 
                boolean lockout = false;
                for (int i=0; i<tiltBufferSize; ++i) {
                    // Check that all tilts in our buffer satisfy the expected angle condition.
                    if (Math.abs(tiltBuffer.get(i) - Calibration.Climber.antennaTouchingAngle) > 
                        Calibration.Climber.antennaTouchingAngleTolerance) {
                        lockout = true;
                        System.out.println("LOCKOUT!!!!!!!!");
                        break;
                    }
                }

                if (buttonCommandedTransition && atTarget() && !lockout) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case HANDOFF_1:
            case HANDOFF_2: {
                this.scheduleIfNotScheduled(handoffCommand);

                if (buttonCommandedTransition && atTarget()) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case REACH_HIGH_RUNG:
            case REACH_TRAVERSAL_RUNG: {
                this.scheduleIfNotScheduled(reachNextCommand);

                if (buttonCommandedTransition && atTarget()) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case PARTIAL_LIFT_TRAVERSAL: {
                this.scheduleIfNotScheduled(partialLiftCommand);

                if (buttonCommandedTransition && atTarget()) {
                    transitionState();
                    buttonCommandedTransition = false;
                }
                break;
            }
            case FINISH: {
                stop();
                break;
            }
            default: {
                stop();
                break;
            }
        }

        if (isClimbing) {
            TrapezoidProfile.State state = trapProfile.calculate(climbTimer.get());
            climberPID.setSetpointPosition(
                state.position,
                isLoaded ? Calibration.Climber.loadedFF.calculate(state.velocity) :
                           Calibration.Climber.unloadedFF.calculate(state.velocity));

            SmartDashboard.putNumber("Climber Setpoint", state.position);
        } else {
            climberPrimaryMotor.stopMotor();
        }

        SmartDashboard.putNumber("Climber Position", filteredPosition);
        SmartDashboard.putNumber("Climber Velocity", filteredVelocity);
        SmartDashboard.putNumber("Robot Y Tilt", curTilt);
        SmartDashboard.putNumber("Robot Y Tilt Rate", curTiltRate);

        SmartDashboard.putString("Current Climber State", currentState.toString());
    }

    public enum ClimberState {
        REST,
        REACH_FIRST_RUNG,
        TOUCH_FIRST_RUNG,
        PARTIAL_LIFT_MID,
        LEAP_OF_FAITH_TO_HIGH,
        HANDOFF_1,
        REACH_HIGH_RUNG,
        PARTIAL_LIFT_HIGH,
        LEAP_OF_FAITH_TO_TRAVRSAL,
        HANDOFF_2,
        REACH_TRAVERSAL_RUNG,
        PARTIAL_LIFT_TRAVERSAL,
        FINISH,
    }
}
