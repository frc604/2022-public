package com._604robotics.quixsam;

import java.util.function.Consumer;

import com._604robotics.quixsam.odometry.SendableOdometryMeasurment;
import com._604robotics.quixsam.vision.SendableVisionMeasurment;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class QuixsamNetworkTable {
    private NetworkTable quixsamTable;
    private NetworkTable visionTable;
    private NetworkTable odometryTable;
    private NetworkTable landmarksTable;
    private NetworkTable prioriTable;
    private NetworkTable estimatesTable;

    public QuixsamNetworkTable(String name, Pose2d priori, Pose2d prioriSigma, Consumer<QuixsamEsimate> estimateListener) {
        quixsamTable = NetworkTableInstance.getDefault().getTable(name);
        visionTable = quixsamTable.getSubTable("vision");
        odometryTable = quixsamTable.getSubTable("odometry");
        landmarksTable = quixsamTable.getSubTable("landmarks");
        prioriTable = quixsamTable.getSubTable("priori");
        estimatesTable = quixsamTable.getSubTable("estimates");

        estimatesTable.addEntryListener((table, key, entry, value, flags) -> {
            estimateListener.accept(new QuixsamEsimate((int) value.getDoubleArray()[0], new Pose2d(value.getDoubleArray()[1], value.getDoubleArray()[2], new Rotation2d(value.getDoubleArray()[3]))));
            // estimatesTable.delete(key);
        }, EntryListenerFlags.kImmediate | EntryListenerFlags.kUpdate);

        Double[] prioriData = new Double[]{(double) 
            0,
            priori.getX(),
            priori.getY(),
            priori.getRotation().getRadians(),
            prioriSigma.getX(),
            prioriSigma.getY(),
            prioriSigma.getRotation().getRadians()
        };
        
        prioriTable.getEntry(String.valueOf(0)).setNumberArray(prioriData);

        // Double[] emptyOdometry = new Double[7];
        // Arrays.fill(emptyOdometry, 0.0);
        // odometryTable.getEntry("0").setNumberArray(emptyOdometry);

        // Double[] emptyVision = new Double[5];
        // Arrays.fill(emptyVision, 0.0);
        // visionTable.getEntry("0").setNumberArray(emptyVision);
    }

    public void addLandmark(int id, Vector3D pose, Vector3D sigmas) {
        Double[] landmarkData = new Double[]{(double) id, pose.getX(), pose.getY(), pose.getZ(), sigmas.getX(), sigmas.getY(), sigmas.getZ()};
        landmarksTable.getEntry(String.valueOf(id)).setNumberArray(landmarkData);
    }

    public void publishOdometry(SendableOdometryMeasurment odometry) {
        Double[] odometryData = new Double[]{(double) 
            odometry.getId(),
            odometry.getPose().getX(),
            odometry.getPose().getY(),
            odometry.getPose().getRotation().getRadians(),
            odometry.getSigmas().getX(),
            odometry.getSigmas().getY(),
            odometry.getSigmas().getRotation().getRadians()
        };

        odometryTable.getEntry(String.valueOf("0")).setNumberArray(odometryData);
    }

    public void publishVision(SendableVisionMeasurment measurment) {
        Double[] visionData = new Double[1+4*measurment.getMeasurments().size()];
        visionData[0] = (double) measurment.getId();
        for (int i = 0; i < measurment.getMeasurments().size(); i++) {
            visionData[4*i + 1] = measurment.getMeasurments().get(i).getFirst(); // bearing
            visionData[4*i + 2] = measurment.getMeasurments().get(i).getSecond(); // elevation
            visionData[4*i + 3] = measurment.getSigmas().get(i).getFirst(); // bearing sigma
            visionData[4*i + 4] = measurment.getSigmas().get(i).getSecond(); // elevation sigma
        }

        visionTable.getEntry(String.valueOf("0")).setNumberArray(visionData);
        
    }

    public void update() {

    }

}
