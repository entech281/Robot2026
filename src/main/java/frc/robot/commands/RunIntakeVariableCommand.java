package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.io.DriveInputSupplier;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeVariableCommand extends EntechCommand {

    private final IntakeSubsystem intakeSS;
    private DriveInputSupplier driveInputSupplier;

    public RunIntakeVariableCommand(IntakeSubsystem intakeSS, DriveInputSupplier driveInputSupplier) {
        this.intakeSS = intakeSS;
        this.driveInputSupplier = driveInputSupplier;
    }

    @Override
    public void execute() {

        DriveInput driveInput = driveInputSupplier.getDriveInput();

        double speed = Math.pow(driveInput.getXSpeed(), 2) + Math.pow(driveInput.getYSpeed(), 2); // was Math.sqrt() of
                                                                                                  // this expression

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
