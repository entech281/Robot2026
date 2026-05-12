package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

public class MultiCameraContainer implements CameraContainerI {
  private final CameraContainerI[] cameraContainers;

  public MultiCameraContainer(CameraContainerI... cameraContainers) {
    this.cameraContainers = cameraContainers;
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

  @Override
  public List<PhotonPipelineResult> getAllUnreadResults() {
    List<PhotonPipelineResult> allResults = new ArrayList<>();
    for (CameraContainerI cameraContainer : cameraContainers) {
      allResults.addAll(cameraContainer.getAllUnreadResults());
    }
    return allResults;
  }

  @Override
  public Optional<List<VisionPose>> getEstimatedPoses() {
    List<VisionPose> allPoses = new ArrayList<>();

    for (CameraContainerI cameraContainer : cameraContainers) {
      Optional<List<VisionPose>> poses = cameraContainer.getEstimatedPoses();
      poses.ifPresent(allPoses::addAll);
    }

    if (allPoses.isEmpty()) {
      return Optional.empty();
    } else {
      return Optional.of(allPoses);
    }
  }
}