package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.entech.subsystems.SubsystemOutput;

public class VisionOutput extends SubsystemOutput {
    private boolean connected = false;

    private List<PhotonPipelineResult> unreadResultsA = new ArrayList<>();
    private List<PhotonPipelineResult> unreadResultsB = new ArrayList<>();
    private List<PhotonPipelineResult> unreadResultsC = new ArrayList<>();
    private List<PhotonPipelineResult> unreadResultsD = new ArrayList<>();

    private List<VisionPose> visionPoses = new ArrayList<>();

    @Override
    public void toLog() {
        for (int i = 0; i < unreadResultsA.size(); i++) {
            Logger.recordOutput("VisionOutput/cameraA/unreadResult_" + i, unreadResultsA.get(i));
        }

        for (int i = 0; i < unreadResultsB.size(); i++) {
            Logger.recordOutput("VisionOutput/cameraB/unreadResult_" + i, unreadResultsB.get(i));
        }

        for (int i = 0; i < unreadResultsC.size(); i++) {
            Logger.recordOutput("VisionOutput/cameraC/unreadResult_" + i, unreadResultsC.get(i));
        }

        for (int i = 0; i < unreadResultsD.size(); i++) {
            Logger.recordOutput("VisionOutput/cameraD/unreadResult_" + i, unreadResultsD.get(i));
        }

        for (int i = 0; i < visionPoses.size(); i++) {
            Logger.recordOutput("VisionOutput/visionPose_" + i, visionPoses.get(i).getPose());
        }

        for (int i = 0; i < visionPoses.size(); i++) {
            Logger.recordOutput("VisionOutput/visionPoseTimeStamps_" + i, visionPoses.get(i).getTimeStamp());
        }

        Logger.recordOutput("VisionOutput/connected", connected);
    }

    /**
     * @return List<PhotonPipelineResult> return the unreadResultsA
     */
    public List<PhotonPipelineResult> getUnreadResultsA() {
        return unreadResultsA;
    }

    /**
     * @param unreadResultsA the unreadResultsA to set
     */
    public void setUnreadResultsA(List<PhotonPipelineResult> unreadResultsA) {
        this.unreadResultsA = unreadResultsA;
    }

    /**
     * @return List<PhotonPipelineResult> return the unreadResultsB
     */
    public List<PhotonPipelineResult> getUnreadResultsB() {
        return unreadResultsB;
    }

    /**
     * @param unreadResultsB the unreadResultsB to set
     */
    public void setUnreadResultsB(List<PhotonPipelineResult> unreadResultsB) {
        this.unreadResultsB = unreadResultsB;
    }

    /**
     * @return List<PhotonPipelineResult> return the unreadResultsC
     */
    public List<PhotonPipelineResult> getUnreadResultsC() {
        return unreadResultsC;
    }

    /**
     * @param unreadResultsC the unreadResultsC to set
     */
    public void setUnreadResultsC(List<PhotonPipelineResult> unreadResultsC) {
        this.unreadResultsC = unreadResultsC;
    }

    /**
     * @return List<PhotonPipelineResult> return the unreadResultsD
     */
    public List<PhotonPipelineResult> getUnreadResultsD() {
        return unreadResultsD;
    }

    /**
     * @param unreadResultsD the unreadResultsD to set
     */
    public void setUnreadResultsD(List<PhotonPipelineResult> unreadResultsD) {
        this.unreadResultsD = unreadResultsD;
    }

    /**
     * @return boolean return the connected
     */
    public boolean isConnected() {
        return connected;
    }

    /**
     * @param connected the connected to set
     */
    public void setConnected(boolean connected) {
        this.connected = connected;
    }

    /**
     * @return List<VisionPose> return the visionPoses
     */
    public List<VisionPose> getVisionPoses() {
        return visionPoses;
    }

    /**
     * @param visionPoses the visionPoses to set
     */
    public void setVisionPoses(List<VisionPose> visionPoses) {
        this.visionPoses = visionPoses;
    }

}