package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConstants;

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
    latestResult = new PhotonPipelineResult(); // Initialize with empty result
    this.robotToCamera = robotToCamera;
  }

  public SoloCameraContainer(String cameraName, Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout, NetworkTableInstance ni) {
    camera = new PhotonCamera(ni, cameraName);
    estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    camera.setDriverMode(false);
    latestResult = new PhotonPipelineResult(); // Initialize with empty result
    this.robotToCamera = robotToCamera;
  }

  @Override
  public PhotonPipelineResult getFilteredResult() {
    var results = camera.getAllUnreadResults();

    if (results.isEmpty()) {
      return latestResult; // Return the last known result if no new ones
    }

    // Get the most recent result and store it
    PhotonPipelineResult result = results.get(results.size() - 1);
    latestResult = result; // Cache for latency access

    List<PhotonTrackedTarget> filteredTargets = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      // Filter by ambiguity
      if (target.getPoseAmbiguity() > RobotConstants.Vision.Filters.MAX_AMBIGUITY) {
        continue;
      }

      // Filter by distance
      if (Math.abs(target.getBestCameraToTarget().getX()) > RobotConstants.Vision.Filters.MAX_DISTANCE) {
        continue;
      }

      // Filter by allowed tag IDs
      boolean allowed = false;
      for (int id : RobotConstants.Vision.Filters.ALLOWED_TAGS) {
        if (target.getFiducialId() == id) {
          allowed = true;
          break;
        }
      }

      if (!allowed) {
        continue;
      }

      filteredTargets.add(target);
    }

    // Create new result with filtered targets
    return new PhotonPipelineResult(
        result.metadata,
        filteredTargets,
        result.getMultiTagResult());
  }

  private PhotonPipelineResult getFilteredResult(PhotonPipelineResult result) {
    List<PhotonTrackedTarget> filteredTargets = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      // Filter by ambiguity
      // if (target.getPoseAmbiguity() > RobotConstants.Vision.Filters.MAX_AMBIGUITY)
      // {
      // continue;
      // }

      // Filter by distance
      // if (Math.abs(target.getBestCameraToTarget().getX()) >
      // RobotConstants.Vision.Filters.MAX_DISTANCE) {
      // continue;
      // }

      // Filter by allowed tag IDs
      // boolean allowed = false;
      // for (int id : RobotConstants.Vision.Filters.ALLOWED_TAGS) {
      // if (target.getFiducialId() == id) {
      // allowed = true;
      // break;
      // }
      // }

      // if (!allowed) {
      // continue;
      // }

      filteredTargets.add(target);
    }

    // Create new result with filtered targets
    return new PhotonPipelineResult(
        result.metadata,
        filteredTargets,
        result.getMultiTagResult());
  }

  @Override
  public Optional<List<VisionPose>> getEstimatedPoses() {
    latestReadResults = camera.getAllUnreadResults();
    List<PhotonPipelineResult> filteredList = new ArrayList<>();

    for (PhotonPipelineResult result : latestReadResults) {
      filteredList.add(getFilteredResult(result));
    }

    List<VisionPose> visionPoses = new ArrayList<>();

    for (PhotonPipelineResult result : filteredList) {
      if (!result.hasTargets()) {
        continue;
      }
      Optional<EstimatedRobotPose> estimatedPose = estimator.estimateCoprocMultiTagPose(result);
      if (!estimatedPose.isPresent()) {
        estimatedPose = estimator.estimateLowestAmbiguityPose(result);
      }

      Logger.recordOutput("SubsystemTest0", estimatedPose.isPresent());
      if (estimatedPose.isPresent()) {
        Logger.recordOutput("SubsystemTest01", estimatedPose.get().timestampSeconds);
        Logger.recordOutput("SubsystemTest1", estimatedPose.get().targetsUsed.size());
        Logger.recordOutput("SubsystemTest2", estimatedPose.get().estimatedPose.toPose2d());
      }

      if (estimatedPose.isPresent()) {
        Pose3d pose = estimatedPose.get().estimatedPose;
        double timeStamp = result.getTimestampSeconds();

        double ambiguity = 0.0;
        for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
          ambiguity += target.getPoseAmbiguity();
        }
        ambiguity /= estimatedPose.get().targetsUsed.size();

        List<Transform3d> camToTargetTransforms = new ArrayList<>();
        for (PhotonTrackedTarget target : estimatedPose.get().targetsUsed) {
          camToTargetTransforms.add(target.getBestCameraToTarget());
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Transform3d transform : camToTargetTransforms) {
          totalDistance += transform.getTranslation().getDistance(robotToCamera.getTranslation());
        }
        double avgDistance = totalDistance / estimatedPose.get().targetsUsed.size();

        visionPoses
            .add(new VisionPose(pose, timeStamp, ambiguity, camera.getName(), estimatedPose.get().targetsUsed,
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
  public Optional<Pose2d> getEstimatedPose() {
    Optional<List<VisionPose>> latestVisionPose = getEstimatedPoses();

    if (latestVisionPose.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(latestVisionPose.get().get(latestVisionPose.get().size() - 1).getPose2d());
  }

  @Override
  public double getLatency() {
    // Access latency from metadata
    if (latestResult != null && latestResult.metadata != null) {
      return latestResult.metadata.getLatencyMillis();
    }
    return 0.0;
  }

  @Override
  public boolean hasTargets() {
    return getFilteredResult().hasTargets();
  }

  @Override
  public int getTargetCount() {
    return getFilteredResult().getTargets().size();
  }

  @Override
  public List<EntechTargetData> getTargetData() {
    List<PhotonTrackedTarget> targets = getFilteredResult().getTargets();
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