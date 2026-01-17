package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSensor;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSensor extends EntechSensor<VisionOutput> {
    
    private final PhotonCamera camera;
    private PhotonPipelineResult result;
    
    public VisionSensor(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }
    
    @Override
    public void initialize() {
        // Nothing needed
    }
    
    @Override
    public void periodic() {
        result = camera.getLatestResult();
    }
    
    @Override
    public boolean isEnabled() {
        return true;
    }
    
    @Override
    public VisionOutput toOutputs() {
        VisionOutput output = new VisionOutput();
        
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
            output.hasTargets = true;
            output.targetId = target.getFiducialId();
            output.yaw = target.getYaw();
            output.pitch = target.getPitch();
            output.area = target.getArea();
            // output.width = target.getMinAreaRectCorners().get(0).distance(target.getMinAreaRectCorners().get(1));
            // output.height = target.getMinAreaRectCorners().get(1).distance(target.getMinAreaRectCorners().get(2));
        }
        
        return output;
    }
    
    @Override
    public Command getTestCommand() {
        return Commands.none();
    }
}