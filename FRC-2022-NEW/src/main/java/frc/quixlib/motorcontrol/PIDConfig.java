package frc.quixlib.motorcontrol;

public class PIDConfig {
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kF = 0.0;

  public PIDConfig(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public PIDConfig(double kP, double kI, double kD, double kF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
  }

  public double getP() {
    return this.kP;
  }

  public double getI() {
    return this.kI;
  }

  public double getD() {
    return this.kD;
  }

  public double getF() {
    return this.kF;
  }
}
