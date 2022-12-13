package com._604robotics.robotnik.motorcontrol.controllers;

public class MotorControllerPIDConfig {
    protected int slot = 0;
    protected double Kp;
    protected double Ki;
    protected double Kd;
    protected double Kf = 0.0;

    public MotorControllerPIDConfig(
        double Kp,
        double Ki,
        double Kd
    ) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public MotorControllerPIDConfig(
            int slot,
            double Kp,
            double Ki,
            double Kd
    ) {
        this.slot = slot;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public MotorControllerPIDConfig(
        double Kp,
        double Ki,
        double Kd,
        double Kf
    ) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public MotorControllerPIDConfig(
            int slot,
            double Kp,
            double Ki,
            double Kd,
            double Kf
    ) {
        this.slot = slot;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public void setSlot(int slot) {
        this.slot = slot;
    }

    public void setP(double Kp) {
        this.Kp = Kp;
    }

    public void setI(double Ki) {
        this.Ki = Ki;
    }

    public void setD(double Kd) {
        this.Kd = Kd;
    }

    public void setF(double Kf) {
        this.Kf = Kf;
    }

    public int getSlot() {
        return this.slot;
    }

    public double getP() {
        return this.Kp;
    }

    public double getI() {
        return this.Ki;
    }

    public double getD() {
        return this.Kd;
    }

    public double getF() {
        return this.Kf;
    }

}
