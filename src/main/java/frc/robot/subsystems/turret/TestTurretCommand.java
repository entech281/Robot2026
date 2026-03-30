package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import frc.entech.commands.EntechCommand;
import frc.entech.util.StoppingCounter;
import frc.robot.RobotConstants;

public class TestTurretCommand extends EntechCommand {
    private final TurretSubsystem turretSubsystem;
    private final StoppingCounter counter = new StoppingCounter(RobotConstants.TEST_CONSTANTS.STANDARD_TEST_LENGTH);

    public TestTurretCommand(TurretSubsystem turretSubsystem) {
        super(turretSubsystem);
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        TurretInput input = new TurretInput();
        input.setRequestedPosition(Degrees.of(0.0));
        turretSubsystem.updateInputs(input);
    }

    @Override
    public void initialize() {
        counter.reset();
        TurretInput input = new TurretInput();
        input.setRequestedPosition(Degrees.of(180.0));
        turretSubsystem.updateInputs(input);
    }

    @Override
    public boolean isFinished() {
        return counter.isFinished(true);
    }
}
