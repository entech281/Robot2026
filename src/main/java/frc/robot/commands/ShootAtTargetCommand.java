package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.entech.commands.EntechCommand;
import frc.entech.util.Triboolean;
import frc.robot.RobotConstants;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.TurretCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShootAtTargetCommand extends EntechCommand {
    private final HoodSubsystem hoodSS;
    private final ShooterSubsystem shooterSS;
    private final TransferSubsystem transferSS;
    private final TurretSubsystem turretSS;
    private Supplier<ShooterCalculator> shooterCalculatorSupplier;
    private Supplier<TurretCalculator> turretCalculatorSupplier;
    private ShooterInput shooterInput = new ShooterInput();
    private HoodInput hoodInput = new HoodInput();
    private TransferInput transferInput = new TransferInput();
    private TurretInput turretInput = new TurretInput();

    public ShootAtTargetCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
            TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem,
            Supplier<TurretCalculator> turretCalculatorSupplier,
            Supplier<ShooterCalculator> shooterCalculatorSupplier) {
        super(hoodSubsystem, shooterSubsystem, transferSubsystem, turretSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.transferSS = transferSubsystem;
        this.turretSS = turretSubsystem;
        this.shooterCalculatorSupplier = shooterCalculatorSupplier;
        this.turretCalculatorSupplier = turretCalculatorSupplier;
    }

    // nothing to end
    @Override
    public void end(boolean interrupted) {
        transferInput = new TransferInput();
        hoodInput = new HoodInput();
        shooterInput = new ShooterInput();

        transferSS.updateInputs(transferInput);
        hoodSS.updateInputs(hoodInput);
        shooterSS.updateInputs(shooterInput);
    }

    @Override
    public void execute() {
        ShotDataRange shotRange = shooterCalculatorSupplier.get().calculateShot();
        double targetTurretAngle = turretCalculatorSupplier.get().calculateTargetTurretAngle();

        ShotData shot = shotRange.getIdealShot();

        shooterInput
        .setSpeed(shot.getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)).in(RPM));
        hoodInput.setRequestedPosition(shot.getHoodAngle().in(Degree));
        turretInput.setRequestedPosition(Degrees.of(targetTurretAngle));

        // shooterInput
        //         .setSpeed(3750);
        // hoodInput.setRequestedPosition(20);
        // turretInput.setRequestedPosition(Degrees.of(45));

        shooterSS.updateInputs(shooterInput);
        hoodSS.updateInputs(hoodInput);
        turretSS.updateInputs(turretInput);

        // boolean turretIsReady = turretCalculatorSupplier.get().isValidTurretAngle(
        // turretSS.getOutputs().getCurrentPosition(),
        // RobotConstants.TURRET.TURRET_POSITION_TOLERANCE_DEGREES);
        // Triboolean shotIsReady = shooterCalculatorSupplier.get().isValidShot(
        // Degrees.of(hoodSS.getOutputs().getHoodMotor().getCurrentPosition()),
        // RPM.of(shooterSS.getOutputs().getShooterMotorA().getCurrentSpeed()),
        // Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS));

        // Triboolean isReadyToShoot = shotIsReady.fand(Triboolean.of(turretIsReady));

        boolean isReadyToShoot = turretSS.getOutputs().isAtRequestedPosition()
                && hoodSS.getOutputs().isAtRequestedPosition()
                && shooterSS.getOutputs().isAtSpeed();

        if (isReadyToShoot) {
            transferInput.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
            transferSS.updateInputs(transferInput);
        } else {
            transferInput.setSpeed(0.0);
            transferSS.updateInputs(transferInput);
        }

        // Logger.recordOutput("TurretCalculatorIsReadyToShoot", turretIsReady);
        // Logger.recordOutput("ShotIsReady", shotIsReady + "");
        // Logger.recordOutput("IsReadyToShoot", isReadyToShoot + "");
        // Logger.recordOutput("ShotRange", shotRange + "");

    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
