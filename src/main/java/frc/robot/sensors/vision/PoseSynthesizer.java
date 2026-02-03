package frc.robot.sensors.vision;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseSynthesizer {

    private MultiCameraContainer multiCamContainer;
    
    public PoseSynthesizer(MultiCameraContainer multiCamContainer) {
        this.multiCamContainer = multiCamContainer;
    }

    public MultiCameraContainer getMultiCamContainer() {
        return multiCamContainer;
    }

    public void setMultiCamContainer(MultiCameraContainer multiCamContainer) {
        this.multiCamContainer = multiCamContainer;
    }

    public Pose2d getEstimatedPose() {
        
        List<PhotonPipelineResult> results = multiCamContainer.getAllUnreadResults();

        results.get(0).getTargets().get(0).getBestCameraToTarget();
        results.get(0).getTargets().get(0).getPoseAmbiguity();


        //TODO: finish
        return new Pose2d();
    }

}
