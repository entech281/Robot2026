package frc.robot.sensors.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

public interface CameraContainerI {
  Optional<List<VisionPose>> getEstimatedPoses();

  double getLatency();

  boolean hasTargets();

  int getTargetCount();

  List<EntechTargetData> getTargetData();

  boolean isDriverMode();

  void setDriverMode(boolean enabled);

  boolean isConnected();

  List<PhotonPipelineResult> getAllUnreadResults();
}