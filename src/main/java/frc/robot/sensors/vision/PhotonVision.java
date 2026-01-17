package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSensor;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
/**
 * Basic PhotonVision subsystem - follows PhotonVision example code
 */
public class PhotonVision extends EntechSensor<VisionOutput> {
    
    public static final String CAMERA_NAME = "OV5647";
 
    
    private PhotonCamera camera;

     @Override
    public void initialize() {
        camera = new PhotonCamera(CAMERA_NAME);

    }

     @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public VisionOutput toOutputs() {
        VisionOutput output = new VisionOutput();
        
        return output;
    }
    


    private VisionOutput getCameraOutput() {
        var result = camera.getLatestResult();
        VisionOutput output = new VisionOutput();

        if (result.hasTargets()) {
            var bestTarget = result.getBestTarget();
            output.setTarget(bestTarget);
        }

        return output;
    }


    @Override
    public Command getTestCommand() {
        return null;
    }

    @Override
    public void simulationPeriodic() {

    }
    
    @Override
    public void periodic() {
    
    }

   
}