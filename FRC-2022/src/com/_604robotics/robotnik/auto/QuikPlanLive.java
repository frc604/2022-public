package com._604robotics.robotnik.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class QuikPlanLive {
  List<Double> timeData = new ArrayList<>();
  List<List<Double>> data = new ArrayList<>();

  NetworkTable quikPlanTable;
  NetworkTable obstaclesTable;
  NetworkTable trajectoryTable;

  public QuikPlanLive() {
    quikPlanTable = NetworkTableInstance.getDefault().getTable("quikplan");
    obstaclesTable = quikPlanTable.getSubTable("obstacles");
    trajectoryTable = quikPlanTable.getSubTable("trajectory");

    System.out.println(NetworkTableInstance.getDefault().isConnected());
  }

  public void publishObstacles(List<Translation2d> obstacles, double obstacleRadius) {
    for (Translation2d obstacle : obstacles) {
      Number[] obstacleData = {obstacle.getX(), obstacle.getY(), obstacleRadius};
      obstaclesTable.getEntry(obstacle.toString()).setNumberArray(obstacleData);
    }
  }

  public boolean isTrajectoryValid() {
    return quikPlanTable.getEntry("Valid_Trajectory").getBoolean(false);
  }

  public void loadTrajectory() {
    clearLoadedData();
    List<Double> keys =
        new ArrayList<String>(trajectoryTable.getKeys())
            .stream()
                .mapToDouble((s) -> Double.parseDouble(s))
                .boxed()
                .collect(Collectors.toList());
    Collections.sort(keys);

    for (Double key : keys) {
      double[] values =
          trajectoryTable.getEntry(String.valueOf(key)).getDoubleArray(new double[11]);
      List<Double> classValues = new ArrayList<>();
      for (Double d : values) {
        classValues.add(d);
      }
      this.timeData.add(key);
      for (TrajectoryState state : TrajectoryState.values()) {
        this.data.get(state.index).add(classValues.get(state.index));
      }
    }
  }

  public List<Double> getState(double time) {
    List<Double> state = new ArrayList<>();
    for (List<Double> dataList : data) {
      LinearInterpolator li = new LinearInterpolator();
      PolynomialSplineFunction psf =
          li.interpolate(
              timeData.stream().mapToDouble(d -> d).toArray(),
              dataList.stream().mapToDouble(d -> d).toArray());
      state.add(psf.value(Math.min(time, getTotalTime())));
    }
    return state;
  }

  public void clearLoadedData() {
    timeData.clear();
    data.clear();

    for (int i = 0; i <= TrajectoryState.values().length - 1; i++) {
      data.add(new ArrayList<Double>());
    }
  }

  public double getTotalTime() {
    return timeData.get(timeData.size() - 1);
  }

  public enum TrajectoryState {
    x(0),
    y(1),
    Theta(2),
    v(3),
    w(4),
    vl(5),
    vr(6),
    al(7),
    ar(8);

    public final int index;

    private TrajectoryState(int index) {
      this.index = index;
    }
  }
}
