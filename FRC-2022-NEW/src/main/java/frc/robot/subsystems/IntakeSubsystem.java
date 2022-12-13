// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  // Intake Motor
  private final QuixTalonFX m_intakeMotor =
      new QuixTalonFX(Constants.Intake.intakeID, Constants.Intake.rollerRatio);

  // Deploy Motor
  private final QuixTalonFX m_deployMotor =
      new QuixTalonFX(Constants.Intake.deployID, Constants.Intake.deployArmRatio);

  private final int kPositionSlot = 0;
  private State m_deployState = new State(m_deployMotor.getSensorPosition(), 0.0);
  private TrapezoidProfile m_deployProfile;
  private final Timer m_deployTimer = new Timer();

  // Intake Subsystem
  public IntakeSubsystem(Mechanism2d simViz) {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Initialize deploy profile and timer
    m_deployProfile =
        new TrapezoidProfile(
            Constants.Intake.deployTrapazoidalConstraints,
            new State(0.5 * Math.PI, 0.0),
            m_deployState);
    m_deployTimer.start();

    // Visualize sim in SmartDashboard.
    m_simViz = simViz;
    setupViz();

    // PID Stuff
    m_deployMotor.setPIDConfig(kPositionSlot, Constants.Intake.deployPIDConfig);
  }

  public void startIntakeSpin() {
    m_intakeMotor.setPercentOutput(-0.5);
  }

  public void deployMethod() {
    m_deployProfile =
        new TrapezoidProfile(
            Constants.Intake.deployTrapazoidalConstraints, new State(0.0, 0.0), m_deployState);
    m_deployTimer.reset();
  }

  public void retractMethod() {
    m_deployProfile =
        new TrapezoidProfile(
            Constants.Intake.deployTrapazoidalConstraints,
            new State(0.5 * Math.PI, 0.0),
            m_deployState);
    m_deployTimer.reset();
  }

  public void stopIntakeSpin() {
    m_intakeMotor.setPercentOutput(0.0);
  }

  @Override
  public void periodic() {
    m_deployState = m_deployProfile.calculate(m_deployTimer.get());
    SmartDashboard.putNumber("Current Position (Deploy Motor)", m_deployMotor.getSensorPosition());
    SmartDashboard.putNumber("Target Position (Deploy Motor)", m_deployState.position);

    double FF = Constants.Intake.deployFF.calculate(m_deployState.position, m_deployState.velocity);
    m_deployMotor.setPositionSetpoint(kPositionSlot, m_deployState.position, FF);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Simulation parameters
  private static final double kArmEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;
  private final SingleJointedArmSim m_deployArmSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          Constants.Intake.deployArmRatio.reduction(),
          SingleJointedArmSim.estimateMOI(
              Constants.Intake.deployArmLength, Constants.Intake.deployArmMass),
          Constants.Intake.deployArmLength,
          0.0, // Min arm angle (radians)
          0.5 * Math.PI, // Max arm angle (radians)
          Constants.Intake.deployArmMass,
          true, // Simulate gravity
          VecBuilder.fill(kArmEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );
  private static final double kRollerEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;
  private final FlywheelSim m_rollerSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          Constants.Intake.rollerRatio.reduction(),
          Constants.Intake.rollerMOI,
          VecBuilder.fill(kRollerEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );

  // Visualization
  private final Mechanism2d m_simViz;
  private MechanismRoot2d m_deployArmPivot;
  private MechanismLigament2d m_deployArm;
  private MechanismRoot2d m_rollerPivot;
  private MechanismLigament2d m_roller;
  private final double kVizArmPivotX = 70.0;
  private final double kVizArmPivotY = 20.0;
  private final double kVizArmLength = 20.0;

  private void setupViz() {
    m_deployArmPivot = m_simViz.getRoot("Deploy Arm Pivot", kVizArmPivotX, kVizArmPivotY);
    m_deployArm =
        m_deployArmPivot.append(
            new MechanismLigament2d(
                "Intake Deploy Arm",
                kVizArmLength,
                Units.radiansToDegrees(m_deployArmSim.getAngleRads()),
                10,
                new Color8Bit(Color.kSilver)));

    m_rollerPivot = m_simViz.getRoot("Intake Roller Pivot", 0, 0);
    m_roller =
        m_rollerPivot.append(
            new MechanismLigament2d("Intake Roller", 5, 90, 10, new Color8Bit(Color.kRed)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Simulate arm and update sensor.
    m_deployArmSim.setInput(
        m_deployMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_deployArmSim.update(TimedRobot.kDefaultPeriod);
    m_deployMotor.setSimSensorPositionAndVelocity(
        m_deployArmSim.getAngleRads(),
        m_deployArmSim.getVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.deployArmRatio);

    // Simulate roller and update sensor.
    m_rollerSim.setInput(
        m_intakeMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_rollerSim.update(TimedRobot.kDefaultPeriod);
    m_intakeMotor.setSimSensorVelocity(
        m_rollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Intake.rollerRatio);

    // Update the arm viz based on the sim.
    double armAngle = m_deployArmSim.getAngleRads();
    m_deployArm.setAngle(Math.toDegrees(armAngle));
    m_rollerPivot.setPosition(
        kVizArmPivotX + kVizArmLength * Math.cos(armAngle),
        kVizArmPivotY + kVizArmLength * Math.sin(armAngle));

    // Update the roller viz based on the sim.
    double kVizScale = 0.2; // Scale viz velocity to make high velocities look better.
    double dRollerAngle =
        Math.toDegrees(m_rollerSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    m_roller.setAngle(m_roller.getAngle() + dRollerAngle * kVizScale);
  }
  // --- END STUFF FOR SIMULATION ---
}
