package com._604robotics.robotnik.motion;

public class SCurveMotionProfile {
  private MotionState initial;
  private MotionState goal;

  private double maxVel;
  private double maxAccel;
  private double avgAccel;

  private double maxJerkAccel;
  private double maxJerkDeccel;

  private double deltaV;

  private double areaAccel;
  private double areaDeccel;

  private double t_1;
  private double t_1c;
  private double t_1l = 0.0;

  private double t_3;
  private double t_3c;
  private double t_3l;

  private double t_2 = 0.0;

  public SCurveMotionProfile(MotionState initial, MotionState goal, MotionConstraints constraints) {
    this.initial = initial;
    this.goal = goal;

    this.maxVel = Math.abs(constraints.maxVelocity);
    this.maxAccel = Math.abs(constraints.maxAcceleration);
    this.avgAccel = Math.abs(constraints.maxJerk);

    calculateParams();
  }

  private void calculateParams() {
    double displacement = goal.position - initial.position;

    if (goal.position < initial.position) {
      maxAccel = -maxAccel;
      avgAccel = -avgAccel;
      maxVel = -maxVel;
    }

    maxJerkAccel =
        (Math.pow(maxAccel, 2) * avgAccel) / ((maxVel - initial.velocity) * (maxAccel - avgAccel));
    maxJerkDeccel =
        (Math.pow(maxAccel, 2) * avgAccel) / ((maxVel - goal.velocity) * (maxAccel - avgAccel));

    t_1 = Math.abs((maxVel - initial.velocity) / avgAccel);
    t_3 = Math.abs((maxVel - goal.velocity) / avgAccel);

    areaAccel = (Math.pow((maxVel - initial.velocity), 2) / (2 * avgAccel));
    areaDeccel = (Math.pow((maxVel - goal.velocity), 2) / (2 * avgAccel));

    if ((Math.abs(areaAccel) + Math.abs(areaDeccel)) > Math.abs(displacement)) {
      maxVel =
          Math.sqrt(
                  Math.abs(
                      (Math.pow(initial.velocity, 2)
                              + 2 * avgAccel * displacement
                              + Math.pow(goal.velocity, 2))
                          / 2))
              * Math.signum(maxVel);

      maxJerkAccel =
          (Math.pow(maxAccel, 2) * avgAccel)
              / ((maxVel - initial.velocity) * (maxAccel - avgAccel));
      maxJerkDeccel =
          (Math.pow(maxAccel, 2) * avgAccel) / ((maxVel - goal.velocity) * (maxAccel - avgAccel));

      areaAccel = (Math.pow((maxVel - initial.velocity), 2) / (2 * avgAccel));
      areaDeccel = (Math.pow((maxVel - goal.velocity), 2) / (2 * avgAccel));

      t_1 = Math.abs((maxVel - initial.velocity) / avgAccel);
      t_3 = Math.abs((maxVel - goal.velocity) / avgAccel);
    } else {
      t_2 =
          Math.abs(
              (Math.abs(displacement) - (Math.abs(areaAccel) + Math.abs(areaDeccel))) / (maxVel));
    }

    t_1c = Math.abs((maxAccel) / (maxJerkAccel));
    t_3c = Math.abs((maxAccel) / (maxJerkDeccel));

    if (t_1c * 2 >= t_1) {
      t_1c = 0.5 * t_1;
    } else {
      t_1l = Math.abs(((maxVel - initial.velocity) / avgAccel) - ((maxAccel) / (maxJerkAccel)));
    }

    if (t_3c * 2 >= t_3) {
      t_3c = 0.5 * t_3;
    } else {
      t_3l = Math.abs(((maxVel - goal.velocity) / avgAccel) - ((maxAccel) / (maxJerkDeccel)));
    }

    System.out.println(t_1c);
  }

  public AccelMotionState sample(double t) {
    double position;
    double velocity;
    double acceleration;

    if (t <= t_1c) {
      velocity = (maxJerkAccel * (Math.pow(t, 2) / 2)) + initial.velocity;
      position = maxJerkAccel * (Math.pow(t, 3) / 6) + initial.position;
      acceleration = maxJerkAccel * t;
    } else if (t_1c < t && t <= t_1l) {
      velocity =
          (Math.pow(maxAccel, 2) / (2 * maxJerkAccel)) + (maxAccel * (t - t_1c)) + initial.velocity;
      position =
          ((maxJerkAccel * Math.pow(t_1c, 3)) / 6)
              + ((maxAccel * Math.pow((t - t_1c), 2)) / 2)
              + ((Math.pow(maxAccel, 2) / (2 * maxJerkAccel)) * (t - t_1c))
              + initial.position;
      acceleration = maxAccel;
    } else if (t_1l < t && t <= t_1) {
      velocity = maxVel - ((maxJerkAccel * Math.pow((t_1 - t), 2)) / 2);
      position =
          (Math.pow((maxVel - initial.velocity), 2) / (2 * avgAccel))
              + ((maxJerkAccel * Math.pow((t_1 - t), 3)) / 6)
              - ((maxVel - initial.velocity) * (t_1 - t))
              + initial.position;
      acceleration = maxAccel - (maxJerkAccel * (t - t_1l));
    } else if (t_1 < t && t <= (t_1 + t_2)) {
      velocity = maxVel;
      position =
          (maxVel) * (t - t_1)
              + (Math.pow((maxVel - initial.velocity), 2) / (2 * avgAccel))
              + initial.position;
      acceleration = 0.0;
    } else if ((t_1 + t_2) < t && t <= (t_1 + t_2 + t_3c)) {
      velocity = maxVel - ((maxJerkDeccel * Math.pow((t_1 + t_2) - t, 2)) / 2);
      position =
          goal.position
              - (Math.pow(maxVel, 2) / (2 * avgAccel))
              + ((maxJerkDeccel * Math.pow(((t_1 + t_2) - t), 3)) / 6)
              - (maxVel * ((t_1 + t_2) - t));
      acceleration = maxAccel - (maxJerkDeccel * (t - t_1l));
    } else if ((t_1 + t_2 + t_3c) < t && t <= (t_1 + t_2 + t_3l)) {
      velocity =
          (Math.pow(maxAccel, 2) / (2 * maxJerkDeccel))
              + (maxAccel * ((t_1 + t_2 + t_3l) - t))
              + goal.velocity;
      position =
          goal.position
              - ((maxJerkDeccel * Math.pow(t_3c, 3)) / 6)
              + ((maxAccel * Math.pow(((t_1 + t_2 + t_3c) - t), 2)) / 2)
              + ((Math.pow(maxAccel, 2) / (2 * maxJerkDeccel)) * ((t_1 + t_2 + t_3l) - t));
      acceleration = maxAccel;
    } else if (t <= getTotalTime()) {
      velocity = (maxJerkDeccel * (Math.pow(t - (getTotalTime()), 2) / 2)) + goal.velocity;
      position = goal.position - maxJerkDeccel * (Math.pow(getTotalTime() - t, 3) / 6);
      acceleration = maxJerkDeccel * t;
    } else {
      position = goal.position;
      velocity = goal.velocity;
      acceleration = 0.0;
    }

    return new AccelMotionState(position, velocity, acceleration);
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
