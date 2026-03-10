package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import static edu.wpi.first.units.Units.Degrees;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Moves the turret by a small step while the command is active.
 * Useful for tuning: hold the button to nudge the turret left/right.
 * Step size is defined in RobotConstants.TURRET.TURRET_JOG_STEP_DEGREES
 */
public class TurretJogCommand extends EntechCommand {
    private final TurretSubsystem turret;
    private final double stepDegrees;

    /**
     * @param turret      the turret subsystem
     * @param stepDegrees positive to move toward + degrees, negative for -
     */
    public TurretJogCommand(TurretSubsystem turret, double stepDegrees) {
        super(turret);
        this.turret = turret;
        this.stepDegrees = stepDegrees;
    }

    @Override
    public void initialize() {
        // no-op on init
    }

    @Override
    public void execute() {
        TurretInput in = new TurretInput();
        Angle current = turret.getOutputs().getCurrentPosition();
        Angle adding = Degrees.of(stepDegrees);
        in.setRequestedPosition(current.plus(adding));
        turret.updateInputs(in);
    }

    @Override
    public void end(boolean interrupted) {
        turret.updateInputs(new TurretInput());
    }

    @Override
    public boolean isFinished() {
        return false; // Run while held
    }
}
