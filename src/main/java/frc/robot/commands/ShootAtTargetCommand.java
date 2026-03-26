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
import frc.robot.util.FlightTimeEstimator;
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
    private boolean shooterReady = false;

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

        shooterReady = false;
    }

    @Override
    public void execute() {
        ShotDataRange shotRange = shooterCalculatorSupplier.get().calculateShotBeta();
        ShotData shot = shotRange.getIdealShot();

        // Pass the hood angle to turret calculator for accurate flight time estimation
        double hoodAngleDeg = shot.getHoodAngle().in(Degree);
        double targetTurretAngle = turretCalculatorSupplier.get().calculateTargetTurretAngle(hoodAngleDeg);

        shooterInput
        .setSpeed(shot.getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)).in(RPM));
        hoodInput.setRequestedPosition(hoodAngleDeg);
        turretInput.setRequestedPosition(Degrees.of(targetTurretAngle));

        shooterSS.updateInputs(shooterInput);
        hoodSS.updateInputs(hoodInput);
        turretSS.updateInputs(turretInput);

        // Log shoot-while-moving telemetry
        TurretCalculator tc = turretCalculatorSupplier.get();
        ShooterCalculator sc = shooterCalculatorSupplier.get();
        double distanceM = tc.getDistanceToTargetMeters();
        double flightTime = FlightTimeEstimator.getFlightTimeSeconds(distanceM, hoodAngleDeg);
        Logger.recordOutput("ShootWhileMoving/FlightTimeSeconds", flightTime);
        Logger.recordOutput("ShootWhileMoving/DistanceMeters", distanceM);
        Logger.recordOutput("ShootWhileMoving/HoodAngleDeg", hoodAngleDeg);
        Logger.recordOutput("ShootWhileMoving/TurretAngleDeg", targetTurretAngle);
        Logger.recordOutput("ShootWhileMoving/RadialVelocityMps", sc.getRadialVelocityMps());
        Logger.recordOutput("ShootWhileMoving/CommandedRPM",
                shot.getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)).in(RPM));

        
        if (!shooterReady) {
            shooterReady = shooterSS.getOutputs().isAtSpeed();
        }

        boolean isReadyToShoot = turretSS.getOutputs().isAtRequestedPosition()
                && hoodSS.getOutputs().isAtRequestedPosition()
                && shooterReady;

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
