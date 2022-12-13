package com._604robotics.robotnik.vision;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera extends VisionCamera {
  private PhotonCamera camera;

  public PhotonVisionCamera(String name, Vector3D pose, double tilt) {
    super(name, pose, tilt);
    camera = new PhotonCamera(name);
  }

  @Override
  public PipelineVisionPacket getLatestMeasurement() {
    PhotonPipelineResult result = camera.getLatestResult();
    List<VisionCamera.Target> targets = new ArrayList<>();
    if (result.hasTargets()) {
      for (PhotonTrackedTarget photonTarget : result.targets) {
        targets.add(
            new Target(
                photonTarget.getYaw(),
                photonTarget.getPitch(),
                photonTarget.getArea(),
                photonTarget.getSkew()));
      }
      return new PipelineVisionPacket(
          result.hasTargets(),
          new Target(
              result.getBestTarget().getYaw(),
              result.getBestTarget().getPitch(),
              result.getBestTarget().getArea(),
              result.getBestTarget().getSkew()),
          targets,
          result.getLatencyMillis());
    } else {
      return new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), 0.0);
    }
  }
}
