package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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

public class ClimberSubsystem extends SubsystemBase {

  private final QuixTalonFX m_climberMotor0 =
      new QuixTalonFX(
          Constants.Climber.climberMotor0,
          new MechanismRatio(),
          Constants.Climber.climberMotorInvert);

  private final QuixTalonFX m_climberMotor1 =
      new QuixTalonFX(
          Constants.Climber.climberMotor1,
          new MechanismRatio(),
          Constants.Climber.climberMotorInvert);

  private final QuixTalonFX m_climberMotor2 =
      new QuixTalonFX(
          Constants.Climber.climberMotor2,
          new MechanismRatio(),
          Constants.Climber.climberMotorInvert);

  public ClimberSubsystem(Mechanism2d simViz) {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);

    // Visualize sim in SmartDashboard.
    m_simViz = simViz;
    setupViz();
  }

  public void climbUp() {
    m_climberMotor0.setPercentOutput(0.5);
    m_climberMotor1.setPercentOutput(0.5);
    m_climberMotor2.setPercentOutput(0.5);
  }

  public void climbDown() {
    m_climberMotor0.setPercentOutput(-0.5);
    m_climberMotor1.setPercentOutput(-0.5);
    m_climberMotor2.setPercentOutput(-0.5);
  }

  public void stop() {
    m_climberMotor0.setPercentOutput(0);
    m_climberMotor1.setPercentOutput(0);
    m_climberMotor2.setPercentOutput(0);
  }

  @Override
  public void periodic() {}

  // --- BEGIN STUFF FOR SIMULATION ---
  // Simulation parameters
  private static final double kElevatorEncoderRadiansPerPulse = 2.0 * Math.PI / 2048;
  private final ElevatorSim m_climberSim =
      new ElevatorSim(
          DCMotor.getFalcon500(1),
          Constants.Climber.climberRatio.reduction(),
          Constants.Climber.carriageMass,
          Constants.Climber.drumRadius,
          Constants.Climber.minHeight,
          Constants.Climber.maxHeight,
          true, // Simulate gravity
          VecBuilder.fill(kElevatorEncoderRadiansPerPulse) // Add noise with a std-dev of 1 tick
          );

  // Visualization
  private final Mechanism2d m_simViz;
  private MechanismRoot2d m_climberRoot;
  private MechanismLigament2d m_climber;

  private void setupViz() {
    m_climberRoot = m_simViz.getRoot("Climber Root", 20, 10);
    m_climber =
        m_climberRoot.append(
            new MechanismLigament2d("Climber", 10, 90, 10, new Color8Bit(Color.kYellow)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // Simulate climber and update sensor & battery voltage.
    m_climberSim.setInput(
        -m_climberMotor0.getPhysicalPercentOutput() * RobotController.getBatteryVoltage());
    m_climberSim.update(TimedRobot.kDefaultPeriod);
    m_climberMotor0.setSimSensorPositionAndVelocity(
        m_climberSim.getPositionMeters(),
        m_climberSim.getVelocityMetersPerSecond(),
        TimedRobot.kDefaultPeriod,
        Constants.Climber.climberRatio);

    // Update viz based on simulated climber.
    final double kHeightPerMeter = 80 / Constants.Climber.maxHeight;
    m_climber.setLength(5 + m_climberSim.getPositionMeters() * kHeightPerMeter);
  }
  // --- END STUFF FOR SIMULATION ---
}
