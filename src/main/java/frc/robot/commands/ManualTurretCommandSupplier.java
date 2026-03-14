package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import frc.entech.commands.EntechCommand;
import frc.robot.io.RobotIO;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.TurretCalculator;

public class ManualTurretCommandSupplier extends EntechCommand {
    private final TurretInput turretInput = new TurretInput();
    private final TurretSubsystem turretSS;
    private Supplier<TurretCalculator> turretCalculator;

    public ManualTurretCommandSupplier(TurretSubsystem turretSubsystem, Supplier<TurretCalculator> turretCalculator) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
        this.turretCalculator = turretCalculator;
    }

    @Override
    public void initialize() {
        turretInput.setRequestedPosition(Degrees.of(turretCalculator.get().calculateTargetTurretAngle()));
        turretSS.updateInputs(turretInput);
    }

    @Override
    public void execute() {
        turretInput.setRequestedPosition(Degrees.of(turretCalculator.get().calculateTargetTurretAngle()));
        turretSS.updateInputs(turretInput);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}