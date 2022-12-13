package com._604robotics.quixsam.vision;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;

public class SendableVisionMeasurment {
    private int id;
    private ArrayList<Pair<Double, Double>> measurments;
    private ArrayList<Pair<Double, Double>> sigmas;

    public SendableVisionMeasurment(int id) {
        this.id = id;
        this.measurments = new ArrayList<>();
        this.sigmas = new ArrayList<>();
    }

    public SendableVisionMeasurment(int id, ArrayList<Pair<Double, Double>> measurments, ArrayList<Pair<Double, Double>> sigmas) {
        this.id = id;
        this.measurments = measurments;
        this.sigmas = sigmas;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public void addMeasurment(Pair<Double, Double> measurment, Pair<Double, Double> sigma) {
        measurments.add(measurment);
        sigmas.add(sigma);
    }

    public ArrayList<Pair<Double, Double>> getMeasurments() {
        return measurments;
    }

    public ArrayList<Pair<Double, Double>> getSigmas() {
        return sigmas;
    }
}
