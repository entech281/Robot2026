package frc.robot.sensors.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.entech.sensors.SensorOutput;
import frc.entech.util.Triboolean;

public class VisionOutput extends SensorOutput {
    private boolean connected = false;

    private List<PhotonPipelineResult> unreadResultsA = new ArrayList<>();
    private List<PhotonPipelineResult> unreadResultsB = new ArrayList<>();
    private List<PhotonPipelineResult> unreadResultsC = new ArrayList<>();
    private List<PhotonPipelineResult> unreadResultsD = new ArrayList<>();

    private List<VisionPose> visionPoses = new ArrayList<>();

    private double averagePoseAmbiguity = 0.0;

    private final double fieldWidth;
    private final double fieldLength;

    private Triboolean goodness;

    public VisionOutput(double fieldWidth, double fieldLength) {
        this.fieldWidth = fieldWidth;
        this.fieldLength = fieldLength;
    }

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
            Logger.recordOutput("VisionOutput/visionPoses/visionPose_" + i + "/pose", visionPoses.get(i).getPose3d());
        }

        for (int i = 0; i < visionPoses.size(); i++) {
            Logger.recordOutput("VisionOutput/visionPoses/visionPose_" + i + "/timestamp",
                    visionPoses.get(i).getTimeStamp());
        }

        for (int i = 0; i < visionPoses.size(); i++) {
            Logger.recordOutput("VisionOutput/visionPoses/visionPose_" + i + "/ambiguity",
                    visionPoses.get(i).getAmbiguity());
        }

        for (int i = 0; i < visionPoses.size(); i++) {
            Logger.recordOutput("VisionOutput/visionPoses/visionPose_" + i + "/cameraUsed",
                    visionPoses.get(i).getCameraUsed());
        }

        Logger.recordOutput("VisionOutput/averagePoseAmbiguity", averagePoseAmbiguity);

        Logger.recordOutput("VisionOutput/connected", connected);

        Logger.recordOutput("VisionOutput/Goodness", goodness.toString());
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

    public Triboolean getGoodness() {
        return goodness;
    }

    public void setGoodness(Triboolean goodness) {
        this.goodness = goodness;
    }

    /**
     * @return double return the averagePoseAmbiguity
     */
    public double getAveragePoseAmbiguity() {
        return averagePoseAmbiguity;
    }

    /**
     * @param averagePoseAmbiguity the averagePoseAmbiguity to set
     */
    public void setAveragePoseAmbiguity(double averagePoseAmbiguity) {
        this.averagePoseAmbiguity = averagePoseAmbiguity;
    }

    public double getFieldWidth() {
        return fieldWidth;
    }

    public double getFieldLength() {
        return fieldLength;
    }
}