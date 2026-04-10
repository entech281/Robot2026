package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeVariableCommand extends EntechCommand {

    private final IntakeSubsystem intakeSS;

    public RunIntakeVariableCommand(IntakeSubsystem intakeSS) {
        this.intakeSS = intakeSS;
    }

    @Override
    public void execute() {

        ChassisSpeeds chassisSpeeds = RobotIO.getInstance().getDriveOutput().getSpeeds();

        double speed = Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2); // was
                                                                                                                    // Math.sqrt()
                                                                                                                    // of
                                                                                                                    // this
                                                                                                                    // expression

        double threshold = LiveTuningHandler.getInstance().getValue("IntakeSubsystem/MinimumVariableSpeed");

        if (speed < threshold) {
            speed = threshold;
        }

        IntakeInput input = new IntakeInput();
        input.setSpeed(speed);
        intakeSS.updateInputs(input);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeInput input = new IntakeInput();
        intakeSS.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
