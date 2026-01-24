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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConstants;

public class SoloCameraContainer implements CameraContainerI {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private PhotonPipelineResult latestResult; // Store the latest result for latency access

  public SoloCameraContainer(String cameraName, Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout) {
    camera = new PhotonCamera(cameraName);
    estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    camera.setDriverMode(false);
    latestResult = new PhotonPipelineResult(); // Initialize with empty result
  }

  public SoloCameraContainer(String cameraName, Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout, NetworkTableInstance ni) {
    camera = new PhotonCamera(ni, cameraName);
    estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    camera.setDriverMode(false);
    latestResult = new PhotonPipelineResult(); // Initialize with empty result
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
        result.getMultiTagResult()
    );
  }

  @Override
  public Optional<Pose2d> getEstimatedPose() {
    PhotonPipelineResult filteredResult = getFilteredResult();
    
    // Pass the filtered result to update()
    Optional<EstimatedRobotPose> estimatedPose = estimator.update(filteredResult);
    
    if (estimatedPose.isPresent()) {
      return Optional.of(estimatedPose.get().estimatedPose.toPose2d());
    } else {
      return Optional.empty();
    }
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
}