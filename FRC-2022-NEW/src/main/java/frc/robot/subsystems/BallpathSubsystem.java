// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.MechanismRatio;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;

public class BallpathSubsystem extends SubsystemBase {
  // Ballpath Motor1
  private final QuixTalonFX m_ballpathTopMotor =
      new QuixTalonFX(Constants.Ballpath.ballPathTopMotor, new MechanismRatio());

  // Ballpath Motor1
  private final QuixTalonFX m_ballpathBottomMotor =
      new QuixTalonFX(Constants.Ballpath.ballPathBottomMotor, new MechanismRatio());

  public BallpathSubsystem(Mechanism2d simViz) {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Visualize sim in SmartDashboard.
    m_simViz = simViz;
    setupViz();
  }

  // Hmm I wonder what this one does
  public void sendToLauncher() {
    m_ballpathTopMotor.setPercentOutput(0.5);
  }

  // For future
  public void recieveFromIntake() {
    // Don't spin the top motor when storing
    m_ballpathBottomMotor.setPercentOutput(-0.5);
  }

  // Same as above
  public void stopSendToLauncher() {
    m_ballpathTopMotor.setPercentOutput(0.0);
    m_ballpathBottomMotor.setPercentOutput(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Simulation parameters
  private static final double kRollerEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;
  private final FlywheelSim m_bottomRollerSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          Constants.Ballpath.rollerRatio.reduction(),
          Constants.Ballpath.rollerMOI,
          VecBuilder.fill(kRollerEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final FlywheelSim m_topRollerSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          Constants.Ballpath.rollerRatio.reduction(),
          Constants.Ballpath.rollerMOI,
          VecBuilder.fill(kRollerEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );

  // Visualization
  private final Mechanism2d m_simViz;
  private MechanismRoot2d m_bottomPivot;
  private MechanismLigament2d m_bottomRoller;
  private MechanismRoot2d m_topPivot;
  private MechanismLigament2d m_topRoller;

  private void setupViz() {
    m_bottomPivot = m_simViz.getRoot("Bottom Roller Pivot", 60, 30);
    m_bottomRoller =
        m_bottomPivot.append(
            new MechanismLigament2d("Bottom Roller", 6, 90, 10, new Color8Bit(Color.kRed)));
    m_topPivot = m_simViz.getRoot("Top Roller Pivot", 50, 50);
    m_topRoller =
        m_topPivot.append(
            new MechanismLigament2d("Top Roller", 6, 90, 10, new Color8Bit(Color.kRed)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Simulate roller and update sensors.
    m_bottomRollerSim.setInput(
        m_ballpathBottomMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_topRollerSim.setInput(
        -m_ballpathTopMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_bottomRollerSim.update(TimedRobot.kDefaultPeriod);
    m_topRollerSim.update(TimedRobot.kDefaultPeriod);
    m_ballpathBottomMotor.setSimSensorVelocity(
        m_bottomRollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Ballpath.rollerRatio);
    m_ballpathTopMotor.setSimSensorVelocity(
        m_topRollerSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Ballpath.rollerRatio);

    // Update viz based on simulated roller velocity.
    double kVizScale = 0.2; // Scale viz velocity to make high velocities look better.
    double dBottomRollerAngle =
        Math.toDegrees(m_bottomRollerSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    m_bottomRoller.setAngle(m_bottomRoller.getAngle() + dBottomRollerAngle * kVizScale);
    double dTopRollerAngle =
        Math.toDegrees(m_topRollerSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    m_topRoller.setAngle(m_topRoller.getAngle() + dTopRollerAngle * kVizScale);
  }
  // --- END STUFF FOR SIMULATION ---
}
