package com._604robotics.robotnik.auto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class QuikPlanReader {
  List<Double> timeData = new ArrayList<>();
  List<List<Double>> data = new ArrayList<>();
  SendableChooser<File> chooser = new SendableChooser<>();

  public QuikPlanReader(Module module) {
    clearLoadedData();
    List<File> files = new ArrayList<>();
    try {
      Files.walk(Filesystem.getDeployDirectory().toPath())
          .map(Path::toFile)
          .filter((file) -> file.getName().endsWith(".csv"))
          .forEach(files::add);
    } catch (IOException e) {
      e.printStackTrace();
    }

    for (File file : files) {
      chooser.addOption(file.getName(), file);
    }

    loadChosenFile();
  }

  public void loadData(File file) {
    clearLoadedData();

    try (BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (!line.isEmpty()) {
          double[] values =
              Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
          List<Double> classValues = new ArrayList<>();
          for (Double d : values) {
            classValues.add(d);
          }
          this.timeData.add(classValues.get(0));
          for (TrajectoryState state : TrajectoryState.values()) {
            this.data.get(state.index).add(classValues.get(state.index + 1));
          }
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void loadChosenFile() {
    loadData(chooser.getSelected());
    System.out.println("Loading: " + chooser.getSelected());
  }

  public void clearLoadedData() {
    timeData.clear();
    data.clear();

    for (int i = 0; i <= TrajectoryState.values().length - 1; i++) {
      data.add(new ArrayList<Double>());
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
