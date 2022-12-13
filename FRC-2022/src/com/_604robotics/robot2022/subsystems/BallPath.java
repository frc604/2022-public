package com._604robotics.robot2022.subsystems;

import com._604robotics.robot2022.Calibration;
import com._604robotics.robot2022.Constants;
import com._604robotics.robot2022.balls.BallManager.BallState;
import com._604robotics.robotnik.devices.FalconEncoder;
import com._604robotics.robotnik.motorcontrol.Motor;
import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.motorcontrol.controllers.TalonPID;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallPath extends SubsystemBase{
    public final QuixTalonFX ballPathBottomMotor = new QuixTalonFX(
        Constants.BallPath.ballPathBottomMotor,
        "Ball Path Bottom  Motor",
        Motor.kFalcon500,
        this
    );

    public final QuixTalonFX ballPathTopMotor = new QuixTalonFX(
        Constants.BallPath.ballPathTopMotor,
        "Ball Path Top Motor",
        Motor.kFalcon500,
        this
    );

    private final FalconEncoder ballPathBottomEncoder = new FalconEncoder(
        ballPathBottomMotor,
        Calibration.BallPath.ballPathBottomRatio
    );

    private final FalconEncoder ballPathTopEncoder = new FalconEncoder(
        ballPathTopMotor,
        Calibration.BallPath.ballPathTopRatio
    );

    private final TalonPID ballPathTopPID = new TalonPID(
        ballPathTopMotor,
        ballPathTopEncoder,
        Calibration.BallPath.ballPathTopPID
    );

    private final TalonPID ballPathTopVelocityPID = new TalonPID(
        ballPathTopMotor,
        ballPathTopEncoder,
        Calibration.BallPath.ballPathTopVelocityPID
    );

    private final TalonPID ballPathBottomPID = new TalonPID(
        ballPathBottomMotor,
        ballPathBottomEncoder,
        Calibration.BallPath.ballPathBottomPID
    );

    private final TalonPID ballPathBottomVelocityPID = new TalonPID(
        ballPathBottomMotor,
        ballPathBottomEncoder,
        Calibration.BallPath.ballPathTopVelocityPID
    );


    public final DigitalInput ballPathIntakeBeamBreak = new DigitalInput(
        Constants.BallPath.ballPathInakeBeamBreak
    );

    private final DigitalInput ballPathBottomBeamBreak = new DigitalInput(
        Constants.BallPath.ballPathBottomBeamBreak
    );

    private final DigitalInput ballPathTopBeamBreak = new DigitalInput(
        Constants.BallPath.ballPathTopBeamBreak
    );

    public final DigitalInput ballPathColorSensor = new DigitalInput(
        Constants.BallPath.ballPathColorSensor
    );

    private final SimpleMotorFeedforward ballPathTopVelcoityFeedforward = Calibration.BallPath.ballPathTopVelcoityFeedforward;

    private final SendableChooser<Boolean> chooser = new SendableChooser<>();

    private BallState topQueue = BallState.NONE;
    private BallState bottomQueue = BallState.NONE;

    private double bottomRollerSetpoint = 0;
    private double topRollerSetpoint = 0;

    private boolean topRollerPowerMode = true;
    private boolean bottomRollerPowerMode = true;

    private boolean intakeBeamBreakTriggered = false;
    private double lastTimeIntakeBeamBreakTriggered = Double.POSITIVE_INFINITY;

    public BallPath() {
        ballPathBottomMotor.setInverted(true);

        ballPathTopEncoder.setdistancePerRotation(0.1595);
        ballPathBottomEncoder.setdistancePerRotation(0.1595);

        // Color sensor returns true when red and false when blue
        chooser.addOption("Red", true);
        chooser.addOption("Blue", false);

        chooser.setDefaultOption("Red", true);

        SmartDashboard.putData("Ball Color", chooser);
    }
    
    public BallState getTopQueue() {
        return topQueue;
    }

    public BallState getBottomQueue() {
        return bottomQueue;
    }

    public boolean getTopBeamBreakTriggered() {
        return !ballPathTopBeamBreak.get();
    }

    public boolean getBottomBeamBreakTriggered() {
        return !ballPathBottomBeamBreak.get();
    }

    public boolean getIntakeBeamBreakTriggered() {
        return !ballPathIntakeBeamBreak.get();
    }

    public void resetBottomBallpathEncoder() {
        ballPathBottomEncoder.zero();

        bottomRollerSetpoint = 0;
    }

    public void resetTopBallpathEncoder() {
        ballPathTopEncoder.zero();

        topRollerSetpoint  = 0;
    }

    public void resetBottomBallpathEncoderWithCurrentPosition() {
        bottomRollerSetpoint = ballPathBottomEncoder.getPosition();
    }

    public void resetTopBallpathEncoderWithCurrentPosition() {
        topRollerSetpoint  = ballPathTopEncoder.getPosition();
    }

    public void setTopPowerMode() {
        topRollerPowerMode = true;
    }
    
    public void setBottomPowerMode() {
        bottomRollerPowerMode = true;
    }

    public boolean colorSensorIsCorrect() {
        // Color sensor returns true when red and false when blue
        return ballPathColorSensor.get() == chooser.getSelected();
    }

    private boolean isCorrectColorRed() {
        // Chooser is true when the selected color is red.
        return chooser.getSelected();
    }

    public int redCount() {
        int count = 0;
        if ((bottomQueue == BallState.CORRECT && isCorrectColorRed()) || (bottomQueue == BallState.INCORRECT && !isCorrectColorRed())) {
            count++;
        }
        if ((topQueue == BallState.CORRECT && isCorrectColorRed()) || (topQueue == BallState.INCORRECT && !isCorrectColorRed())) {
            count++;
        }
        return count;
    }

    public int blueCount() {
        int count = 0;
        if ((bottomQueue == BallState.CORRECT && !isCorrectColorRed()) || (bottomQueue == BallState.INCORRECT && isCorrectColorRed())) {
            count++;
        }
        if ((topQueue == BallState.CORRECT && !isCorrectColorRed()) || (topQueue == BallState.INCORRECT && isCorrectColorRed())) {
            count++;
        }
        return count;
    }

    public boolean hasIncorrectBallInTriangle() {
        if (!getBottomBeamBreakTriggered() && (Timer.getFPGATimestamp() - lastTimeIntakeBeamBreakTriggered > 0.05)) {
            return !colorSensorIsCorrect();
        }
        return false;
    }

    public void updateBottomQueue() {
        if (getBottomBeamBreakTriggered()) {
            if (colorSensorIsCorrect()) {
                bottomQueue = BallState.CORRECT;
            } else {
                bottomQueue = BallState.INCORRECT;
            }
        } else {
            bottomQueue = BallState.NONE;
        }
    }

    public void transitionBottomQueueToTopQueue() {
        topQueue = bottomQueue;
        bottomQueue = BallState.NONE;
    }

    public void emptyBottomQueue() {
        bottomQueue = BallState.NONE;
    }

    public void emptyTopQueue() {
        topQueue = BallState.NONE;
    }

    public void setTopQueue(BallState topQueue) {
        this.topQueue = topQueue;
    }

    public void setBottomQueue(BallState bottomQueue) {
        this.bottomQueue = bottomQueue;
    }

    public void stop() {
        topRollerPowerMode = false;
        bottomRollerPowerMode = false;

        // ballPathTopMotor.stopMotor();
        // ballPathBottomMotor.stopMotor();
    }

    public void runForIntake() {
        bottomRollerPowerMode = true;
        ballPathBottomMotor.set(Calibration.BallPath.intakePower);
    }

    public void ejectFromIntake() {
        bottomRollerPowerMode = true;
        ballPathBottomMotor.set(-Calibration.BallPath.intakeEjectPower);
    }

    public void yeetFromIntake() {
        bottomRollerPowerMode = true;
        ballPathBottomMotor.set(-Calibration.BallPath.intakeYeetPower);

        topRollerPowerMode = true;
        ballPathTopMotor.set(-Calibration.BallPath.intakeYeetPower);
    }

    public void moveAndKeepBallInTopPosition() {
        bottomRollerPowerMode = true;
        ballPathBottomVelocityPID.setSetpointVelocity(0.010, ballPathTopVelcoityFeedforward.calculate(0.010));

        topRollerPowerMode = true;
        ballPathTopVelocityPID.setSetpointVelocity(0.010, ballPathTopVelcoityFeedforward.calculate(0.010));
    }

    public void moveUpBottomRoller() {
        bottomRollerPowerMode = false;
        bottomRollerSetpoint = 0.1;

        topRollerPowerMode = false;
        topRollerSetpoint = 0.15;
    }

    public void runForIntakeEject() {
        bottomRollerPowerMode = true;
        ballPathBottomMotor.set(-0.7);
    }

    public boolean launchBall() {
        // Feed slowly when there is a ball near the launcher and quickly otherwise.
        double feedSpeed = getTopBeamBreakTriggered() ?
            Calibration.BallPath.shootFeedSpeedSlow : Calibration.BallPath.shootFeedSpeedFast;

        bottomRollerPowerMode = true;
        ballPathBottomPID.setSetpointVelocity(
            feedSpeed,
            ballPathTopVelcoityFeedforward.calculate(feedSpeed));

        topRollerPowerMode = true;
        ballPathTopVelocityPID.setSetpointVelocity(
            feedSpeed,
            ballPathTopVelcoityFeedforward.calculate(feedSpeed));

        boolean isEmpty = !getTopBeamBreakTriggered() && !getBottomBeamBreakTriggered();
        return isEmpty;
    }

    public int getNumberOfBalls() {
        int count = 0;
        if (getTopQueue() != BallState.NONE) {
            count++;
        }
        if (getBottomQueue() != BallState.NONE) {
            count++;
        }

        return count;
    }


    @Override
    public void periodic() {
        if (!intakeBeamBreakTriggered && getIntakeBeamBreakTriggered()) {
            intakeBeamBreakTriggered = true;
            lastTimeIntakeBeamBreakTriggered = Timer.getFPGATimestamp();
        } else if (!getIntakeBeamBreakTriggered()) {
            intakeBeamBreakTriggered = false;
            lastTimeIntakeBeamBreakTriggered = Double.POSITIVE_INFINITY;
        }

        if (!topRollerPowerMode) {
            ballPathTopPID.setSetpointPosition(topRollerSetpoint);
        }

        if (!bottomRollerPowerMode) {
            ballPathBottomPID.setSetpointPosition(bottomRollerSetpoint);
        }
        
        SmartDashboard.putString("Bottom Queue", getBottomQueue().toString());
        SmartDashboard.putString("Top Queue", getTopQueue().toString());

        SmartDashboard.putBoolean("Intake Beam Break", !ballPathIntakeBeamBreak.get());
        SmartDashboard.putBoolean("Bottom Beam Break", !ballPathBottomBeamBreak.get());
        SmartDashboard.putBoolean("Top Beam Break", !ballPathTopBeamBreak.get());
        SmartDashboard.putBoolean("Color Sensor", !ballPathColorSensor.get());
    }
}
