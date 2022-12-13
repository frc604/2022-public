package com._604robotics.robotnik.devices;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class ADIS16470 implements Sendable {
    private static ADIS16470 single_instance = null;
    private final ADIS16470_IMU imu;

    public static ADIS16470 getInstance() {
    if (single_instance == null) single_instance = new ADIS16470();

    return single_instance;
    }

    public ADIS16470() {
    imu = new ADIS16470_IMU();
    imu.configCalTime(CalibrationTime._8s);
    imu.setYawAxis(IMUAxis.kZ);

    SendableRegistry.addLW(this, "ADIS16470");
    }

    public void reset() {
    imu.reset();
    }

    public void calibrate() {
    imu.calibrate();
    }

    public void calibrate(CalibrationTime time) {
    imu.configCalTime(time);
    imu.calibrate();
    }

    public double getAngle() {
    return -imu.getAngle();
    }

    public double getRate() {
    return -imu.getRate();
    }

    public double getYAccel() {
    return imu.getAccelY();
    }

    public double getXAccel() {
    return imu.getAccelX();
    }

    public double getZAccel() {
        return imu.getAccelZ();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getAngle, null);
    }
}