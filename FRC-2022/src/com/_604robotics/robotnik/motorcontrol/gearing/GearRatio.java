package com._604robotics.robotnik.motorcontrol.gearing;

public class GearRatio implements CalculableRatio {
  public double driving = 1;
  public double driven = 1;

  public GearRatio(double drivingTeeth, double drivenTeeth) {
    this.driving = drivingTeeth;
    this.driven = drivenTeeth;
  }

  @Override
  public double calculate(double input) {
    return input * (double) (driving / driven);
  }
}
