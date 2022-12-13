package frc.quixlib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.quixlib.math.MathUtils;
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

public class QuikPlanSwerveTrajectoryReader {
  private double m_totalTime = 0.0;
  private final List<PolynomialSplineFunction> m_interpolatedStateData = new ArrayList<>();
  private final LinearInterpolator m_interpolator = new LinearInterpolator();
  private final SendableChooser<File> m_chooser = new SendableChooser<>();
  private File m_selectedFile = null;
  private final Field2d m_fieldViz;

  public QuikPlanSwerveTrajectoryReader(Field2d fieldViz) {
    m_fieldViz = fieldViz;

    // Read trajectory files
    final List<File> files = new ArrayList<>();
    try {
      Files.walk(Filesystem.getDeployDirectory().toPath())
          .map(Path::toFile)
          .filter((file) -> file.getName().endsWith(".csv"))
          .forEach(files::add);
    } catch (IOException e) {
      e.printStackTrace();
    }

    // Add option for each file
    for (File file : files) {
      m_chooser.addOption(file.getName(), file);
    }

    // Select and load first file by default
    if (files.size() > 0) {
      File firstFile = files.get(0);
      m_chooser.setDefaultOption(firstFile.getName(), firstFile);
      SmartDashboard.putData(m_chooser);
      loadSelectedFile();
    }
  }

  /** Loads the selected file if it has changed. */
  public void loadSelectedFile() {
    final File selectedFile = m_chooser.getSelected();
    if (selectedFile == m_selectedFile) {
      return;
    }
    loadTrajectory(selectedFile);
    m_selectedFile = selectedFile;
    SmartDashboard.putString("Selected Trajectory", selectedFile.getName());
  }

  private void loadTrajectory(File file) {
    clearLoadedTrajectory();

    // Temp data.
    final ArrayList<Double> timeData = new ArrayList<>();
    final ArrayList<ArrayList<Double>> stateData = new ArrayList<ArrayList<Double>>();
    for (int i = 0; i < CSVState.values().length; i++) {
      stateData.add(new ArrayList<Double>());
    }

    // Read time and state data.
    try (final BufferedReader br = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          continue;
        }
        final double[] data =
            Arrays.stream(line.split(",")).mapToDouble(Double::parseDouble).toArray();
        timeData.add(data[0]);
        for (int i = 1; i < data.length; i++) {
          stateData.get(i - 1).add(data[i]);
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
      return;
    }

    // Save interpolated state data.
    for (List<Double> dataList : stateData) {
      m_interpolatedStateData.add(
          m_interpolator.interpolate(
              timeData.stream().mapToDouble(d -> d).toArray(),
              dataList.stream().mapToDouble(d -> d).toArray()));
    }
    m_totalTime = timeData.get(timeData.size() - 1);

    // Update viz
    final double kNumSamples = 100;
    final var vizPoses = new ArrayList<Pose2d>();
    for (int i = 0; i < kNumSamples; i++) {
      double t = getTotalTime() * i / kNumSamples;
      vizPoses.add(MathUtils.convertPoseToCenterOfField(getState(t).getPose()));
    }
    m_fieldViz.getObject("traj").setPoses(vizPoses);
  }

  private void clearLoadedTrajectory() {
    m_totalTime = 0.0;
    m_interpolatedStateData.clear();
  }

  private List<Double> getInterpolatedCSVState(double time) {
    List<Double> state = new ArrayList<>();
    for (PolynomialSplineFunction psf : m_interpolatedStateData) {
      state.add(psf.value(Math.min(time, getTotalTime())));
    }
    return state;
  }

  public QuikplanTrajectoryState getState(double time) {
    final var state = getInterpolatedCSVState(time);
    return new QuikplanTrajectoryState(
        new Pose2d(
            state.get(CSVState.x.index),
            state.get(CSVState.y.index),
            new Rotation2d(state.get(CSVState.theta.index))),
        state.get(CSVState.dx.index),
        state.get(CSVState.dy.index),
        state.get(CSVState.dTheta.index));
  }

  public Pose2d getInitialPose() {
    return getState(0.0).getPose();
  }

  public double getTotalTime() {
    return m_totalTime;
  }

  // TODO: generalize
  public boolean doShoot(double time) {
    return getInterpolatedCSVState(time).get(CSVState.shoot.index) == 1.0;
  }

  // TODO: generalize
  public boolean doYeet(double time) {
    return getInterpolatedCSVState(time).get(CSVState.yeet.index) == 1.0;
  }

  private enum CSVState {
    // Common across years
    x(0),
    y(1),
    theta(2),
    dx(3),
    dy(4),
    dTheta(5),
    // Year-specific
    // TODO: generalize
    shoot(6),
    yeet(7);

    public final int index;

    private CSVState(int index) {
      this.index = index;
    }
  }

  public class QuikplanTrajectoryState {
    private Pose2d m_pose;
    private double m_dx;
    private double m_dy;
    private double m_dTheta;

    public QuikplanTrajectoryState(Pose2d pose, double dx, double dy, double dTheta) {
      m_pose = pose;
      m_dx = dx;
      m_dy = dy;
      m_dTheta = dTheta;
    }

    public Pose2d getPose() {
      return m_pose;
    }

    public double getXVelRef() {
      return m_dx;
    }

    public double getYVelRef() {
      return m_dy;
    }

    public double getThetaVelRef() {
      return m_dTheta;
    }
  }
}
