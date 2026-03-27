package frc.robot.commands;

import frc.entech.commands.EntechCommand;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.intake.IntakeInput;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Runs the intake at a configured speed in the specified direction.
 * Useful for tuning: press button to run intake/outtake.
 */
public class RunIntakeCommand extends EntechCommand {
    private final IntakeSubsystem intakeSS;
    private boolean direction;
    private static final String KEY_STRING = "IntakeSubsystem/SetSpeed";

    public RunIntakeCommand(IntakeSubsystem intake) {
        this(intake, true);
    }

    /**
     * @param intake    the intake subsystem
     * @param direction true for intake, false for outtake
     */
    public RunIntakeCommand(IntakeSubsystem intake, boolean direction) {
        super(intake);
        intakeSS = intake;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        IntakeInput input = new IntakeInput();
        double speed = LiveTuningHandler.getInstance().getValue(KEY_STRING);
        if (!direction) {
            input.setSpeed(-speed);
        } else {
            input.setSpeed(speed);
        }
        intakeSS.updateInputs(input);
    }

    @Override
    public void execute() {
        IntakeInput input = new IntakeInput();
        double speed = LiveTuningHandler.getInstance().getValue(KEY_STRING);
        if (direction) {
            input.setSpeed(speed);
        } else {
            input.setSpeed(-speed);
        }
        intakeSS.updateInputs(input);
    }

    public void runIntake() {
        IntakeInput input = new IntakeInput();
        input.setSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSS.updateInputs(new IntakeInput());
    }

    @Override
    public boolean isFinished() {
        return false; // Run while held
    }
}