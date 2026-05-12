package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SoloCameraContainer implements CameraContainerI {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final Transform3d robotToCamera;
  private PhotonPipelineResult latestResult; // Store the latest result for latency access
  private List<PhotonPipelineResult> latestReadResults = new ArrayList<>();

  public SoloCameraContainer(String cameraName, Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout) {
    camera = new PhotonCamera(cameraName);
    estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    camera.setDriverMode(false);
    latestResult = new PhotonPipelineResult();
    this.robotToCamera = robotToCamera;
  }

  public SoloCameraContainer(String cameraName, Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout, NetworkTableInstance ni) {
    camera = new PhotonCamera(ni, cameraName);
    estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    camera.setDriverMode(false);
    latestResult = new PhotonPipelineResult();
    this.robotToCamera = robotToCamera;
  }

  private Optional<EstimatedRobotPose> calculateEstimatedPose(PhotonPipelineResult result) {
    Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
    if (!estimate.isPresent()) {
      estimate = estimator.estimateLowestAmbiguityPose(result);
    }
    return estimate;
  }

  private double calculateVisionPoseAmbiguity(List<PhotonTrackedTarget> targets) {
    double ambiguity = 0.0;
    for (PhotonTrackedTarget target : targets) {
      ambiguity += target.getPoseAmbiguity();
    }
    ambiguity /= targets.size();
    return ambiguity;
  }

  private double calculateVisionPoseAverageDistance(List<PhotonTrackedTarget> targets) {
    List<Transform3d> camToTargetTransforms = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      camToTargetTransforms.add(target.getBestCameraToTarget());
    }

    double totalDistance = 0.0;
    for (Transform3d transform : camToTargetTransforms) {
      totalDistance += transform.getTranslation().getDistance(robotToCamera.getTranslation());
    }
    return totalDistance / targets.size();
  }

  @Override
  public Optional<List<VisionPose>> getEstimatedPoses() {
    latestReadResults = camera.getAllUnreadResults();
    List<PhotonPipelineResult> filteredList = new ArrayList<>();

    for (PhotonPipelineResult result : latestReadResults) {
      if (result.hasTargets()) {
        filteredList.add(result);
      }
      latestResult = result;
    }

    List<VisionPose> visionPoses = new ArrayList<>();

    for (PhotonPipelineResult result : filteredList) {
      Optional<EstimatedRobotPose> estimatedPose = calculateEstimatedPose(result);
      if (estimatedPose.isPresent()) {
        EstimatedRobotPose estimate = estimatedPose.get();

        double ambiguity = calculateVisionPoseAmbiguity(estimate.targetsUsed);
        double avgDistance = calculateVisionPoseAverageDistance(estimate.targetsUsed);

        visionPoses
            .add(new VisionPose(estimate.estimatedPose, result.getTimestampSeconds(), ambiguity, camera.getName(),
                estimate.targetsUsed,
                avgDistance));
      }
    }

    if (visionPoses.isEmpty()) {
      return Optional.empty();
    } else {
      return Optional.of(visionPoses);
    }
  }

  @Override
  public double getLatency() {
    if (latestResult != null && latestResult.metadata != null) {
      return latestResult.metadata.getLatencyMillis();
    }
    return 0.0;
  }

  @Override
  public boolean hasTargets() {
    return latestResult.hasTargets();
  }

  @Override
  public int getTargetCount() {
    return latestResult.getTargets().size();
  }

  @Override
  public List<EntechTargetData> getTargetData() {
    List<PhotonTrackedTarget> targets = latestResult.getTargets();
    List<Integer> targetIds = new ArrayList<>();

    for (PhotonTrackedTarget target : targets) {
      targetIds.add(target.getFiducialId());
    }

    List<EntechTargetData> data = new ArrayList<>();
    data.add(new EntechTargetData(targetIds, camera.getName()));
    return data;
  }

  @Override
  public boolean isDriverMode() {
    return camera.getDriverMode();
  }

  @Override
  public void setDriverMode(boolean enabled) {
    camera.setDriverMode(enabled);
  }

  @Override
  public boolean isConnected() {
    return camera.isConnected();
  }

  @Override
  public List<PhotonPipelineResult> getAllUnreadResults() {
    return latestReadResults;
  }
}