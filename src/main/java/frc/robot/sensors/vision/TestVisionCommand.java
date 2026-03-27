package frc.robot.sensors.vision;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;

public class TestVisionCommand extends EntechCommand {
    private static final int TARGET_FIDUCIAL_ID = 30;
    private int stage = 0;

    @Override
    public void execute() {
        switch (stage) {
            case 0:
                Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
                        "PUT TAG " + TARGET_FIDUCIAL_ID + " IN-FRONT CAMERA A");
                if (checkResults(RobotIO.getInstance().getVisionOutput().getUnreadResultsA())) {
                    stage = 1;
                }
                break;
            case 1:
                Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
                        "PUT TAG " + TARGET_FIDUCIAL_ID + " IN-FRONT CAMERA B");
                if (checkResults(RobotIO.getInstance().getVisionOutput().getUnreadResultsB())) {
                    stage = 2;
                }
            case 2:
                Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
                        "PUT TAG " + TARGET_FIDUCIAL_ID + " IN-FRONT CAMERA C");
                if (checkResults(RobotIO.getInstance().getVisionOutput().getUnreadResultsC())) {
                    stage = 3;
                }
            case 3:
                Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
                        "PUT TAG " + TARGET_FIDUCIAL_ID + " IN-FRONT CAMERA D");
                if (checkResults(RobotIO.getInstance().getVisionOutput().getUnreadResultsD())) {
                    stage = 4;
                }
            default:
                break;
        }
    }

    private boolean checkResults(List<PhotonPipelineResult> results) {
        if (results.isEmpty()) {
            return false;
        } else {
            if (results.get(0).hasTargets()) {
                return results.get(0).getTargets().get(0).fiducialId == TARGET_FIDUCIAL_ID;
            } else {
                return false;
            }
        }
    }

    @Override
    public void initialize() {
        stage = 0;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput(RobotConstants.OperatorMessages.SUBSYSTEM_TEST,
                "VISION TEST COMPLETE");
    }

    @Override
    public boolean isFinished() {
        return stage > 3;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
