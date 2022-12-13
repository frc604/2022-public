package frc.quixlib.localization;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;

public class NTVisionMeasurement {
  private int m_id;
  private ArrayList<Integer> m_targetIDs;
  private ArrayList<Pair<Double, Double>> m_measurements;
  private ArrayList<Pair<Double, Double>> m_sigmas;

  public NTVisionMeasurement(int id) {
    this.m_id = id;
    this.m_targetIDs = new ArrayList<>();
    this.m_measurements = new ArrayList<>();
    this.m_sigmas = new ArrayList<>();
  }

  // public NTVisionMeasurement(
  //     int id,
  //     ArrayList<Pair<Double, Double>> measurements,
  //     ArrayList<Pair<Double, Double>> sigmas) {
  //   this.m_id = id;
  //   this.m_measurements = measurements;
  //   this.m_sigmas = sigmas;
  // }

  public void setId(int id) {
    this.m_id = id;
  }

  public void addMeasurement(
      int targetID, Pair<Double, Double> measurement, Pair<Double, Double> sigma) {
    m_targetIDs.add(targetID);
    m_measurements.add(measurement);
    m_sigmas.add(sigma);
  }

  public double[] toArray() {
    double[] data = new double[1 + 5 * m_measurements.size()];
    data[0] = (double) m_id;
    for (int i = 0; i < m_measurements.size(); i++) {
      data[5 * i + 1] = (double) m_targetIDs.get(i); // target ID
      data[5 * i + 2] = m_measurements.get(i).getFirst(); // bearing
      data[5 * i + 3] = m_measurements.get(i).getSecond(); // elevation
      data[5 * i + 4] = m_sigmas.get(i).getFirst(); // bearing sigma
      data[5 * i + 5] = m_sigmas.get(i).getSecond(); // elevation sigma
    }
    return data;
  }
}
