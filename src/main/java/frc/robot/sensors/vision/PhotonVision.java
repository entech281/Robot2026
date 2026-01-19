package frc.robot.sensors.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
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
        output = getCameraOutput();
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
        periodic();
    }
    
    @Override
    public void periodic() {
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
        
            }
        }
    }

   
}