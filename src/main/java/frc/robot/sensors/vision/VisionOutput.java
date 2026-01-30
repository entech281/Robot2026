package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import frc.entech.subsystems.SubsystemOutput;

public class VisionOutput extends SubsystemOutput {

    private boolean isDriverMode = false;
    private boolean isConnected = true;
    private boolean hasTargets = false;
    private Optional<Pose2d> estimatedPose = Optional.empty();

    private List<PhotonPipelineResult> unreadResults = new ArrayList<>();

    @Override
    public void toLog() {
        Logger.recordOutput("VisionOutput/isDriverMode", isDriverMode);
        Logger.recordOutput("VisionOutput/isConnected", isConnected);
        Logger.recordOutput("VisionOutput/hasTargets", hasTargets);
        for (int i = 0; i < unreadResults.size(); i++) {
            Logger.recordOutput("VisionOutput/unreadResult_" + i, unreadResults.get(i));
        }
    }

    /**
     * @return boolean return the isDriverMode
     */
    public boolean isDriverMode() {
        return isDriverMode;
    }

    /**
     * @param isDriverMode the isDriverMode to set
     */
    public void setIsDriverMode(boolean isDriverMode) {
        this.isDriverMode = isDriverMode;
    }

    /**
     * @return boolean return the isConnected
     */
    public boolean isConnected() {
        return isConnected;
    }

    /**
     * @param isConnected the isConnected to set
     */
    public void setIsConnected(boolean isConnected) {
        this.isConnected = isConnected;
    }

    /**
     * @return boolean return the hasTargets
     */
    public boolean hasTargets() {
        return hasTargets;
    }

    /**
     * @param hasTargets the hasTargets to set
     */
    public void setHasTargets(boolean hasTargets) {
        this.hasTargets = hasTargets;
    }

    /**
     * @return List<PhotonPipelineResult> return the unreadResults
     */
    public List<PhotonPipelineResult> getUnreadResults() {
        return unreadResults;
    }

    /**
     * @param unreadResults the unreadResults to set
     */
    public void setUnreadResults(List<PhotonPipelineResult> unreadResults) {
        this.unreadResults = unreadResults;
    }

    /**
     * @return Optional<Pose2d> return the estimatedPose
     */
    public Optional<Pose2d> getEstimatedPose() {
        return estimatedPose;
    }

    /**
     * @param estimatedPose the estimatedPose to set
     */
    public void setEstimatedPose(Optional<Pose2d> estimatedPose) {
        this.estimatedPose = estimatedPose;
    }

}