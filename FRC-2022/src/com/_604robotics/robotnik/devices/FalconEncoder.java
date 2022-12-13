package com._604robotics.robotnik.devices;

import com._604robotics.robotnik.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FalconEncoder implements IntegratedEncoder {
  private final WPI_TalonFX talon;

  private CalculableRatio ratio;

  private double conversionFactor = 1.0;
  private boolean inverted = false;

  public FalconEncoder(QuixTalonFX talon) {
    this.talon = talon.getController();
    ratio = null;
  }

  public FalconEncoder(QuixTalonFX talon, CalculableRatio ratio) {
    this.talon = talon.getController();
    this.ratio = ratio;
    conversionFactor = ratio.calculate(1.0);
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
      conversionFactor = distancePerRotation * (1.0 / 2048.0);
    } else {
      conversionFactor = ratio.calculate(distancePerRotation * (1.0 / 2048.0));
    }
  }

  @Override
  public double getPositionConversionFactor() {
    return conversionFactor;
  }

  @Override
  public double getVelocityConversionFactor() {
    return conversionFactor * (1000.0 / 1.0);
  }
  
  @Override
  public void zero() {
    zero(0.0);
  }
  
  @Override
  public void zero(double value) {
    talon.setSelectedSensorPosition(value / conversionFactor);
  }

  @Override
  public double getPosition() {
    double factor;

    if (inverted) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }

    return (talon.getSelectedSensorPosition() * factor * conversionFactor);
  }
  
  @Override
  public double getVelocity() {
    double factor;

    if (inverted) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }

    return talon.getSelectedSensorVelocity() * factor * conversionFactor * (1000.0 / 100.0);
  }

  // Sets the Falcon integrated encoder in sim.
  // Converts to Falcon native units.
  @Override
  public void setSimPosition(double value) {
    int nativeUnits = (int) Math.round((inverted ? -value : value) / conversionFactor);
    this.talon.getSimCollection().setIntegratedSensorRawPosition(nativeUnits);
  }
}
