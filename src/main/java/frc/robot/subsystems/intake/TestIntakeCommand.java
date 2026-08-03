package frc.robot.subsystems.intake;

import frc.entech.commands.EntechCommand;
import frc.entech.util.StoppingCounter;
import frc.robot.RobotConstants;

public class TestIntakeCommand extends EntechCommand {
    private final IntakeSubsystem intake;
    private final StoppingCounter counter = new StoppingCounter(RobotConstants.TEST_CONSTANTS.STANDARD_TEST_LENGTH);

    public TestIntakeCommand(IntakeSubsystem intake) {
        super(intake);
        this.intake = intake;
    }

    @Override
    public void initialize() {
        counter.reset();
        IntakeInput in = new IntakeInput();
        in.setSpeed(0.5);
        intake.acceptInputs(in);
    }

    @Override
    public void end(boolean interrupted) {
        IntakeInput in = new IntakeInput();
        in.setSpeed(0.0);
        intake.acceptInputs(in);
    }

    @Override
    public boolean isFinished() {
        return counter.isFinished(true);
    }
}
