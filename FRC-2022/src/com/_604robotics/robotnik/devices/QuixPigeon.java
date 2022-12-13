package com._604robotics.robotnik.devices;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

public class QuixPigeon {
    private PigeonIMU pigeon;

    public QuixPigeon(int id) {
        pigeon = new PigeonIMU(id);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 20);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 20);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 20);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 1000);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 1000);
    }

    public boolean isReady() {
        return pigeon.getState() == PigeonIMU.PigeonState.Ready;
    }

    public void setYaw(double yaw) {
        if (isReady()) {
            pigeon.setYaw(yaw);
        } else {
            System.out.println("Pigeon is not ready!");
        }
    }

    public double getYawContinous() {
        if (isReady()) {
            return pigeon.getYaw();
        } else {
            System.out.println("Pigeon is not ready!");
            return 0.0;
        }
    }

    public double getYawNonContinous() {
        if (isReady()) {
            double[] ypr = new double[3];
            pigeon.getYawPitchRoll(ypr);

            return ypr[0];
        } else {
            System.out.println("Pigeon is not ready!");
            return 0.0;
        }
    }
}
