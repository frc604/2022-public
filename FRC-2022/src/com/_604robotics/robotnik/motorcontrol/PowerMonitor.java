package com._604robotics.robotnik.motorcontrol;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import java.util.HashMap;
import java.util.Map;

public class PowerMonitor {
  private static PowerMonitor single_instance = null;

  // public final PowerDistribution panel;
  public  ModuleType MType;
  // public final Compressor compressor;
  public PneumaticsModuleType PMType;

  private final NetworkTableInstance network = NetworkTableInstance.getDefault();
  private final NetworkTable table = network.getTable("powermonitor");

  private HashMap<String, QuixMotorController> controllers = new HashMap<>();  

  public int PDchannel;

  public PowerMonitor(int PDchannel, ModuleType MType, int CompressorID, PneumaticsModuleType PMType) {
    this.PDchannel = PDchannel;

    // panel = new PowerDistribution(PDchannel, MType);
    // compressor = new Compressor(CompressorID, PMType);

    // table.getEntry("Compressor Current").setDouble(compressor.getCurrent());
    // table.getEntry("Compressor State").setBoolean(compressor.enabled());
    // table.getEntry("Is Compressed").setBoolean(compressor.getPressureSwitchValue());
  }

  /**
   * @param name
   * @return MotorController
   */
  public QuixMotorController getController(String name) {
    return controllers.getOrDefault(name, null);
  }

  /**
   * @param controller
   * @param name
   */
  public void addController(QuixMotorController controller, String name) {
    controllers.put(name, controller);
    
    table
        .getSubTable(controller.getSubsystem().getName())
        .getSubTable(name)
        .getEntry("Current")
        .setDouble(controller.getOutputCurrent());
    table
        .getSubTable(controller.getSubsystem().getName())
        .getSubTable(name)
        .getEntry("Limit")
        .setDouble(controller.getCurrentLimit());
    table
        .getSubTable(controller.getSubsystem().getName())
        .getSubTable(name)
        .getEntry("Limiting")
        .setBoolean(controller.isCurrentLimiting());
  }

  public void update() {
    for (Map.Entry<String, QuixMotorController> mapElement : controllers.entrySet()) {
      String name = mapElement.getKey();
      QuixMotorController controller = mapElement.getValue();

      table
          .getSubTable(controller.getSubsystem().getName())
          .getSubTable(name)
          .getEntry("Current")
          .setDouble(controller.getOutputCurrent());
      table
          .getSubTable(controller.getSubsystem().getName())
          .getSubTable(name)
          .getEntry("Limit")
          .setDouble(controller.getCurrentLimit());
      table
          .getSubTable(controller.getSubsystem().getName())
          .getSubTable(name)
          .getEntry("Limiting")
          .setBoolean(controller.isCurrentLimiting());
    }


    // table.getEntry("Compressor Current").setDouble(compressor.getCurrent());
    // table.getEntry("Compressor State").setBoolean(compressor.enabled());
    // table.getEntry("Is Compressed").setBoolean(compressor.getPressureSwitchValue());

    //checkFaults();
  }

  /** @param state */
  public void updateCompressor(boolean state) {
    // if (state) {
    //   compressor.enableDigital();
    // } else {
    //   compressor.disable();
    // }
  }

  @SuppressWarnings("unused")
  /* private void checkFaults() {
    if (compressor.getCompressorCurrentTooHighFault()) {
      logger.warning("Compressor overcurrent fault!");
    } else if (compressor.getCompressorNotConnectedFault()) {
      logger.warning("Compressor is not connected!");
    } else if (compressor.getCompressorShortedFault()) {
      logger.warning("Compressor was shorted!");
    }
  } */

  /**
   * @param PDPchannel
   * @param CompressorID
   * @return PowerMonitor
   */
  public static PowerMonitor getInstance(int PDPchannel, ModuleType ModuleType, int CompressorID, PneumaticsModuleType PneumaticsModuleType) {
    if (single_instance == null) single_instance = new PowerMonitor(PDPchannel, ModuleType, CompressorID, PneumaticsModuleType);

    return single_instance;
  }

  public static PowerMonitor getInstance() {
    if (single_instance == null) single_instance = new PowerMonitor(1, ModuleType.kRev, 0, PneumaticsModuleType.REVPH); 

    return single_instance;
  }
}
