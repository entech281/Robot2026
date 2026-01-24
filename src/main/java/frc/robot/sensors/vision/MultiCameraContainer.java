package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import frc.entech.util.EntechGeometryUtils;

public class MultiCameraContainer implements CameraContainerI {
  private final CameraContainerI[] cameraContainers;

  public MultiCameraContainer(CameraContainerI... cameraContainers) {
    this.cameraContainers = cameraContainers;
  }

  @Override
  public Optional<Pose2d> getEstimatedPose() {
    List<Pose2d> estimatedPoses = new ArrayList<>();
    
    for (CameraContainerI cameraContainer : cameraContainers) {
      Optional<Pose2d> estPose = cameraContainer.getEstimatedPose();
      if (estPose.isPresent()) {
        estimatedPoses.add(estPose.get());
      }
    }
    
    if (estimatedPoses.isEmpty())
      return Optional.empty();
    
    if (estimatedPoses.size() == 1)
      return Optional.of(estimatedPoses.get(0));
    
    // Average multiple camera poses
    Pose2d averagePose = estimatedPoses.get(0);
    for (int i = 1; i < estimatedPoses.size(); i++) {
      averagePose = EntechGeometryUtils.averagePose2d(averagePose, estimatedPoses.get(i));
    }
    
    return Optional.of(averagePose);
  }

  @Override
  public PhotonPipelineResult getFilteredResult() {
    List<PhotonTrackedTarget> allTargets = new ArrayList<>();
    
    for (CameraContainerI cameraContainer : cameraContainers) {
      PhotonPipelineResult result = cameraContainer.getFilteredResult();
      allTargets.addAll(result.getTargets());
    }
    
    // Return a simple result with all combined targets
    // Since we can't easily merge metadata, just return targets
    PhotonPipelineResult combined = new PhotonPipelineResult();
    // Note: This creates an empty result - targets will need to be accessed via individual cameras
    // This is a limitation of the multi-camera approach with the new API
    return combined;
  }

  @Override
  public double getLatency() {
    double latencySum = 0.0;
    for (CameraContainerI cameraContainer : cameraContainers) {
      latencySum += cameraContainer.getLatency();
    }
    return latencySum / cameraContainers.length;
  }

  @Override
  public int getTargetCount() {
    int targetCount = 0;
    for (CameraContainerI cameraContainer : cameraContainers) {
      targetCount += cameraContainer.getTargetCount();
    }
    return targetCount;
  }

  @Override
  public boolean hasTargets() {
    for (CameraContainerI cameraContainer : cameraContainers) {
      if (cameraContainer.hasTargets()) {
        return true;
      }
    }
    return false;
  }

  @Override
  public List<EntechTargetData> getTargetData() {
    List<EntechTargetData> data = new ArrayList<>();
    for (CameraContainerI cameraContainer : cameraContainers) {
      data.addAll(cameraContainer.getTargetData());
    }
    return data;
  }

  @Override
  public boolean isDriverMode() {
    for (CameraContainerI cameraContainer : cameraContainers) {
      if (cameraContainer.isDriverMode()) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void setDriverMode(boolean enabled) {
    for (CameraContainerI cameraContainer : cameraContainers) {
      cameraContainer.setDriverMode(enabled);
    }
  }

  @Override
  public boolean isConnected() {
    for (CameraContainerI cameraContainer : cameraContainers) {
      if (!cameraContainer.isConnected()) {
        return false;
      }
    }
    return true;
  }
}