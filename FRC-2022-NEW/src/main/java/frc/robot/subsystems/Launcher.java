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

public class Launcher extends SubsystemBase {

  // creates the 2 motor controller objects that control the front and rear flywheels, respectively
  private final QuixTalonFX launcherFrontMotor =
      new QuixTalonFX(
          Constants.Launcher
              .launcherFrontMotorID, // CAN ID (you can think of it as like port the motor is
          // connected to)
          new MechanismRatio() // ratio between driving and driven pulleys
          );
  private final QuixTalonFX launcherBackMotor =
      new QuixTalonFX(
          Constants.Launcher.launcherRearMotorID,
          new MechanismRatio(),
          Constants.Launcher.launcherFrontMotorInvert);

  // add subsystem methods here
  public void spinRoller() {
    launcherFrontMotor.setPercentOutput(0.25);
    launcherBackMotor.setPercentOutput(0.25);
  }

  public void stopRoller() {
    launcherFrontMotor.setPercentOutput(0.0);
    launcherBackMotor.setPercentOutput(0.0);
  }

  public Launcher(Mechanism2d simViz) {
    // Show scheduler status in SmartDashboard
    SmartDashboard.putData(this);

    // Visualize sim in SmartDashboard.
    m_simViz = simViz;
    setupViz();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  // Simulation parameters
  private static final double kFlywheelEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;
  private final FlywheelSim m_frontFlywheelSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          Constants.Launcher.flywheelRatio.reduction(),
          Constants.Launcher.MOI,
          VecBuilder.fill(kFlywheelEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final FlywheelSim m_rearFlywheelSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1),
          Constants.Launcher.flywheelRatio.reduction(),
          Constants.Launcher.MOI,
          VecBuilder.fill(kFlywheelEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );

  // Visualization
  private final Mechanism2d m_simViz;
  private MechanismRoot2d m_frontPivot;
  private MechanismLigament2d m_frontFlywheel;
  private MechanismRoot2d m_rearPivot;
  private MechanismLigament2d m_rearFlywheel;

  private void setupViz() {
    m_frontPivot = m_simViz.getRoot("FlywheelFrontPivot", 30, 70);
    m_frontFlywheel =
        m_frontPivot.append(
            new MechanismLigament2d("Front Flywheel", 5, 90, 10, new Color8Bit(Color.kBlue)));
    m_rearPivot = m_simViz.getRoot("FlywheelRearPivot", 50, 70);
    m_rearFlywheel =
        m_rearPivot.append(
            new MechanismLigament2d("Rear Flywheel", 5, 90, 10, new Color8Bit(Color.kBlue)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Simulate launcher flywheels and update sensors.
    m_frontFlywheelSim.setInput(
        launcherFrontMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_rearFlywheelSim.setInput(
        launcherBackMotor.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_frontFlywheelSim.update(TimedRobot.kDefaultPeriod);
    m_rearFlywheelSim.update(TimedRobot.kDefaultPeriod);
    launcherFrontMotor.setSimSensorVelocity(
        m_frontFlywheelSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Launcher.flywheelRatio);
    launcherBackMotor.setSimSensorVelocity(
        m_rearFlywheelSim.getAngularVelocityRadPerSec(),
        TimedRobot.kDefaultPeriod,
        Constants.Launcher.flywheelRatio);

    // Update the viz based on the simulated flywheel velocity.
    double kVizScale = 0.2; // Scale viz velocity to make high velocities look better.
    double dAngleFront =
        Math.toDegrees(
            m_frontFlywheelSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    m_frontFlywheel.setAngle(m_frontFlywheel.getAngle() + dAngleFront * kVizScale);
    double dAngleRear =
        Math.toDegrees(m_rearFlywheelSim.getAngularVelocityRadPerSec() * TimedRobot.kDefaultPeriod);
    m_rearFlywheel.setAngle(m_rearFlywheel.getAngle() + dAngleRear * kVizScale);
  }
  // --- END STUFF FOR SIMULATION ---
}
