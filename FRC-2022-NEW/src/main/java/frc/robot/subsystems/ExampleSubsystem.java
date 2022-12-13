// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;

public class ExampleSubsystem extends SubsystemBase {
  private final QuixTalonFX m_motor =
      new QuixTalonFX(Constants.Example.motorID, Constants.Example.motorRatio);

  public ExampleSubsystem() {
    // Show scheduler status in SmartDashboard.
    SmartDashboard.putData(this);
  }

  public void spinForwards() {
    m_motor.setPercentOutput(1.0);
  }

  public void spinReverse() {
    m_motor.setPercentOutput(-1.0);
  }

  public void stop() {
    m_motor.setPercentOutput(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  // --- END STUFF FOR SIMULATION ---
}
