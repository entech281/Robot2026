package frc.robot.subsystems.shooter;

import frc.entech.commands.EntechCommand;
import frc.entech.util.StoppingCounter;
import frc.robot.RobotConstants;

public class TestShooterCommand extends EntechCommand {
    private final ShooterSubsystem shooterSubsystem;
    private final StoppingCounter counter = new StoppingCounter(RobotConstants.TEST_CONSTANTS.STANDARD_TEST_LENGTH * 5);

    public TestShooterCommand(ShooterSubsystem shooterSubsystem) {
        super(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        ShooterInput input = new ShooterInput();
        input.setSpeed(0.0);
        shooterSubsystem.updateInputs(input);
    }

    @Override
    public void initialize() {
        counter.reset();
        ShooterInput input = new ShooterInput();
        input.setSpeed(4000.0);
        shooterSubsystem.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return counter.isFinished(true);
    }
}
