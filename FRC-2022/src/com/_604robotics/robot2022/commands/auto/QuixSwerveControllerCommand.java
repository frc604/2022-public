package com._604robotics.robot2022.commands.auto;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com._604robotics.robot2022.balls.BallManager;
import com._604robotics.robot2022.commands.YeetFromIntake;
import com._604robotics.robot2022.subsystems.Swerve;
import com._604robotics.robotnik.auto.FalconDashboard;
import com._604robotics.robotnik.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.auto.QuikPlanSwerveReader.TrajectoryState;
import com._604robotics.robotnik.swerve.QuixHolonomicDriveController;
import com._604robotics.robotnik.swerve.QuixSwerveDriveKinematics;
import com._604robotics.robotnik.swerve.QuixSwerveModuleState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class QuixSwerveControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final QuikPlanSwerveReader reader;
  private final Swerve swerve;
  private final BallManager ballManager;
  private final Consumer<Boolean> yeet;
  private final Supplier<Pose2d> pose;
  private final Supplier<Rotation2d> continousYaw;
  private final QuixSwerveDriveKinematics kinematics;
  private final QuixHolonomicDriveController controller;
  private final Consumer<QuixSwerveModuleState[]> outputModuleStates;

 
  public QuixSwerveControllerCommand(
    QuikPlanSwerveReader reader,
    BallManager ballManager,
    Consumer<Boolean> yeet,
    Swerve swerve,
    Supplier<Pose2d> pose,
    Supplier<Rotation2d> continousYaw,
    QuixSwerveDriveKinematics kinematics,
    QuixHolonomicDriveController controller,
    Consumer<QuixSwerveModuleState[]> outputModuleStates) {
    this.reader = reader;
    this.swerve = swerve;
    this.ballManager = ballManager;
    this.yeet = yeet;
    this.pose = pose;
    this.continousYaw = continousYaw; /// NEED a seperate continous yaw from gyro because odometry wraps...
    this.kinematics = kinematics;
    this.controller = controller;
    this.outputModuleStates = outputModuleStates;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = timer.get();
    var desiredState = reader.getState(curTime);

    Pose2d pathPose = new Pose2d( // Target Pose
      desiredState.get(TrajectoryState.x.index),
      desiredState.get(TrajectoryState.y.index),
      new Rotation2d(desiredState.get(TrajectoryState.Theta.index))
    );

    var targetChassisSpeeds = controller.calculate(
        pose.get(), // Current Pose
        continousYaw.get(),
        pathPose,
        desiredState.get(TrajectoryState.dx.index),
        desiredState.get(TrajectoryState.dy.index),
        desiredState.get(TrajectoryState.dTheta.index)
    );

    FalconDashboard.getInstance().publishPathPose(pathPose);

    var targetModuleStates = kinematics.toQuixSwerveModuleStates(targetChassisSpeeds);

    // SmartDashboard.putNumber("Target X", desiredState.get(TrajectoryState.x.index));
    // SmartDashboard.putNumber("Current X", pose.get().getX());

    // SmartDashboard.putNumber("Target Y", desiredState.get(TrajectoryState.y.index));
    // SmartDashboard.putNumber("Current Y", pose.get().getY());

    // SmartDashboard.putNumber("Target Yaw", desiredState.get(TrajectoryState.Theta.index));
    // SmartDashboard.putNumber("Current Yaw", pose.get().getRotation().getRadians());

    if (reader.doShoot(curTime)) {
      ballManager.setShootAuto(true);
      ballManager.setIntakeAuto(false);

      yeet.accept(false);
    } else if (reader.doYeet(curTime)) {
      ballManager.forceIntakeLatch(false);

      ballManager.setShootAuto(false);
      ballManager.setIntakeAuto(false);
      
      yeet.accept(true);
    } else {
      ballManager.setShootAuto(false);
      ballManager.setIntakeAuto(true);

      yeet.accept(false);
    }

    outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    ballManager.setShootAuto(false);
    ballManager.setIntakeAuto(false);

    yeet.accept(false);

    double endYaw = Units.radiansToDegrees(reader.getState(reader.getTotalTime()).get(TrajectoryState.Theta.index));

    swerve.zeroContinousYaw(endYaw);

    var targetChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    var targetModuleStates = kinematics.toQuixSwerveModuleStates(targetChassisSpeeds);

    outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(reader.getTotalTime());
  }
}