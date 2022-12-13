package com._604robotics.robotnik.devices;

import com._604robotics.robotnik.motorcontrol.QuixSparkMAX;
import com._604robotics.robotnik.motorcontrol.gearing.CalculableRatio;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NEOEncoder implements IntegratedEncoder {
  private final RelativeEncoder encoder;

  private CalculableRatio ratio;

  private boolean inverted = false;

  public NEOEncoder(QuixSparkMAX spark) {
    encoder = spark.controller.getEncoder();
    ratio = null;
  }

  public NEOEncoder(QuixSparkMAX spark, CalculableRatio ratio) {
    encoder = spark.controller.getEncoder();
    this.ratio = ratio;
  }

  @Override
  public boolean getInverted() {
    return inverted;
  }

  @Override
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }
  
  @Override
  public void setdistancePerRotation(double distancePerRotation) {
    if (ratio == null) {
      encoder.setPositionConversionFactor(distancePerRotation);
      encoder.setVelocityConversionFactor(distancePerRotation * (1.0 / 60.0));
    } else {
      encoder.setPositionConversionFactor(ratio.calculate(distancePerRotation));
      SmartDashboard.putNumber("REUDCC", ratio.calculate(1.0));
      System.out.println(
          "Soarks are dumb " + (ratio.calculate(distancePerRotation) * (1.0 / 60.0)));
      encoder.setVelocityConversionFactor(ratio.calculate(distancePerRotation) * (1.0 / 60.0));
    }
  }

  @Override
  public double getPositionConversionFactor() {
    return encoder.getPositionConversionFactor();
  }

  @Override
  public double getVelocityConversionFactor() {
    return encoder.getVelocityConversionFactor();
  }

  @Override
  public void zero() {
    encoder.setPosition(0.0);
  }

  @Override
  public void zero(double value) {
    encoder.setPosition(value);
  }

  @Override
  public double getPosition() {
    int factor;

    if (inverted) {
      factor = -1;
    } else {
      factor = 1;
    }

    return encoder.getPosition() * factor;
  }

  @Override
  public double getVelocity() {
    int factor;

    if (inverted) {
      factor = -1;
    } else {
      factor = 1;
    }

    return encoder.getVelocity() * factor;
  }

  @Override
  public void setSimPosition(double value) {
    // TODO: Implement
  }
}
