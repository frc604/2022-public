package com._604robotics.robotnik.motorcontrol;

/* Modified from https://github.com/Bankst/PowerManagedRobot */

public class Motor {

  public final double freeSpeed;
  public final double freeCurrent;
  public final double stallTorque;
  public final double stallCurrent;

  public final double r;
  public final double kv;
  public final double kt;

  public int numMotors = 1;

  public Motor(
      double freeSpeed,
      double freeCurrent,
      double stallTorque,
      double stallCurrent,
      double r,
      double kv,
      double kt) {
    this.freeSpeed = freeSpeed;
    this.freeCurrent = freeCurrent;
    this.stallTorque = stallTorque;
    this.stallCurrent = stallCurrent;

    this.r = r;
    this.kv = kv;
    this.kt = kt;
  }

  public Motor(double freeSpeed, double freeCurrent, double stallTorque, double stallCurrent) {
    this.freeSpeed = freeSpeed;
    this.freeCurrent = freeCurrent;
    this.stallTorque = stallTorque;
    this.stallCurrent = stallCurrent;

    r = 12 / stallCurrent;
    kv = freeSpeed / (12 - freeCurrent * r);
    kt = stallTorque / stallCurrent;
  }

  public double predictiveCurrentLimit(double currentV, double maxI, double currentRPM) {
    double currentI = currentV / r - currentRPM / (kv * r);
    double outputV = currentV;

    if (currentI > maxI) {
      outputV = maxI * r + currentRPM / kv;
    }
    return outputV;
  }

  public double reactiveCurrentLimit(double currentV, double maxI, double currentI) {
    double omega = kv * currentV - currentI * r * kv;
    double outputV = currentV;

    if (currentI > maxI) {
      outputV = maxI * r + omega / kv;
    }
    return outputV;
  }

  public void setNumber(int _number) {
    this.numMotors = _number;
  }

  public Motor dual() {
    this.setNumber(2);
    return this;
  }

  public Motor triple() {
    this.setNumber(2);
    return this;
  }

  public static final Motor kCIM = new Motor(5330, 2.7, 2.41, 131, 0.0916, 453.514, 0.0184);
  public static final Motor kMiniCIM = new Motor(5840, 3, 1.41, 89);
  public static final Motor kNEO = new Motor(5880, 1.3, 3.36, 166);
  public static final Motor kNEO550 = new Motor(11000, 1.4, 0.97, 100);
  public static final Motor kFalcon500 = new Motor(6380, 1.5, 4.69, 257);
  public static final Motor kBAG = new Motor(13180, 1.8, 0.43, 53);
  public static final Motor k775Pro = new Motor(18730, 0.7, 0.71, 134);
  public static final Motor kAndyMark9015 = new Motor(14720, 3.7, 0.36, 71);
  public static final Motor kAndyMarkNeveRest = new Motor(5480, 0.4, 0.17, 10);
  public static final Motor kRS775125 = new Motor(5800, 1.6, 0.28, 18);
  public static final Motor kRS77518V = new Motor(13050, 2.7, 0.72, 97);
  public static final Motor kRS550 = new Motor(19000, 0.4, 0.38, 84);
}
