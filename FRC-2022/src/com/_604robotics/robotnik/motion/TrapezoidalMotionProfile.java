package com._604robotics.robotnik.motion;

public class TrapezoidalMotionProfile {
  private MotionState initial;
  private MotionState goal;

  private double maxVel;
  private double maxAccel;

  private double areaAccel;
  private double areaDeccel;

  private double t_1;
  private double t_3;
  private double t_2 = 0.0;

  public TrapezoidalMotionProfile(
      MotionState initial, MotionState goal, MotionConstraints constraints) {
    this.initial = initial;
    this.goal = goal;

    this.maxVel = Math.abs(constraints.maxVelocity);
    this.maxAccel = Math.abs(constraints.maxAcceleration);

    calculateParams();
  }

  private void calculateParams() {
    double displacement = goal.position - initial.position;

    if (goal.position < initial.position) {
      maxAccel = -maxAccel;
      maxVel = -maxVel;
    }

    t_1 = Math.abs((maxVel - initial.velocity) / maxAccel);
    t_3 = Math.abs((maxVel - goal.velocity) / maxAccel);

    areaAccel = areaTrapezoid(initial.velocity, maxVel, t_1);
    areaDeccel = areaTrapezoid(goal.velocity, maxVel, t_3);

    if ((Math.abs(areaAccel) + Math.abs(areaDeccel)) > Math.abs(displacement)) {
      maxVel =
          Math.sqrt(
                  Math.abs(
                      (Math.pow(initial.velocity, 2)
                              + 2 * maxAccel * displacement
                              + Math.pow(goal.velocity, 2))
                          / 2))
              * Math.signum(maxVel);

      areaAccel = areaTrapezoid(initial.velocity, maxVel, t_1);
      areaDeccel = areaTrapezoid(goal.velocity, maxVel, t_3);

      t_1 = (maxVel - initial.velocity) / (maxAccel);
      t_3 = (maxVel - goal.velocity) / (maxAccel);
    } else {
      t_2 =
          Math.abs(
              (Math.abs(displacement) - (Math.abs(areaAccel) + Math.abs(areaDeccel))) / (maxVel));
    }
  }

  public MotionState sample(double t) {
    double position;
    double velocity;

    if (t <= t_1) {
      velocity = maxAccel * t + initial.velocity;
      position = areaTrapezoid(initial.velocity, velocity, t) + initial.position;
    } else if (t_1 < t && t <= (t_1 + t_2)) {
      velocity = maxVel;
      position = areaAccel + (velocity * (t - t_1)) + initial.position;
    } else if ((t_1 + t_2) < t && t <= (getTotalTime())) {
      velocity = -maxAccel * (t - (t_1 + t_2)) + maxVel;
      position =
          areaTrapezoid(maxVel, velocity, t - (t_1 + t_2))
              + areaTrapezoid(initial.velocity, maxVel, t_1)
              + (maxVel * t_2)
              + initial.position;
    } else {
      position = goal.position;
      velocity = goal.velocity;
    }

    return new MotionState(position, velocity);
  }

  public double getTotalTime() {
    return t_2 + t_1 + t_3;
  }

  public double getAccelPeriod() {
    return t_1;
  }

  public double getCruisePeriod() {
    return t_2;
  }

  public double getDeccelPeriod() {
    return t_3;
  }

  public boolean getInverted() {
    return maxVel < 0.0;
  }

  private double areaTrapezoid(double b, double b_1, double h) {
    return 0.5 * (b + b_1) * h;
  }

  public void setParams(MotionState initial, MotionState goal, MotionConstraints constraints) {
    this.initial = initial;
    this.goal = goal;

    this.maxVel = Math.abs(constraints.maxVelocity);
    this.maxAccel = Math.abs(constraints.maxAcceleration);

    calculateParams();
  }
}
