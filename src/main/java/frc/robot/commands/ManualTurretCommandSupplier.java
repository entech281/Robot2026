package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import frc.entech.commands.EntechCommand;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ManualTurretCommandSupplier extends EntechCommand {
    private final TurretInput turretInput = new TurretInput();
    private final TurretSubsystem turretSS;
    private Supplier<Double> turretAngleSupplier;

    public ManualTurretCommandSupplier(TurretSubsystem turretSubsystem, Supplier<Double> turretAngleSupplier) {
        super(turretSubsystem);
        this.turretSS = turretSubsystem;
        this.turretAngleSupplier = turretAngleSupplier;
    }

    @Override
    public void initialize() {
        turretInput.setRequestedPosition(Degrees.of(turretAngleSupplier.get()));
        turretSS.updateInputs(turretInput);
    }

    @Override
    public void execute() {
        turretInput.setRequestedPosition(Degrees.of(turretAngleSupplier.get()));
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