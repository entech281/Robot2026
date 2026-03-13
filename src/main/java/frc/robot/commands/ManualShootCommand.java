package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.entech.commands.EntechCommand;
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

public class ManualShootCommand extends EntechCommand {
    private final HoodSubsystem hoodSS;
    private final ShooterSubsystem shooterSS;
    private final TransferSubsystem transferSS;
    private final TurretSubsystem turretSS;
    private ShooterInput shooterInput = new ShooterInput();
    private HoodInput hoodInput = new HoodInput();
    private TransferInput transferInput = new TransferInput();
    private TurretInput turretInput = new TurretInput();
    private Angle turretAngle;
    private AngularVelocity shooterSpeed;
    private Angle hoodAngle;

    public ManualShootCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
            TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem, Angle turretAngle,
            AngularVelocity shooterSpeed, Angle hoodAngle) {
        super(hoodSubsystem, shooterSubsystem, transferSubsystem, turretSubsystem);
        this.hoodSS = hoodSubsystem;
        this.shooterSS = shooterSubsystem;
        this.transferSS = transferSubsystem;
        this.turretSS = turretSubsystem;
        this.turretAngle = turretAngle;
        this.shooterSpeed = shooterSpeed;
        this.hoodAngle = hoodAngle;
    }

    public ManualShootCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem,
            TransferSubsystem transferSubsystem, TurretSubsystem turretSubsystem, Angle turretAngle,
            Supplier<ShooterCalculator> shooterCalculator) {
        this(shooterSubsystem, hoodSubsystem, transferSubsystem, turretSubsystem, turretAngle,
                shooterCalculator.get().calculateShot().getIdealShot()
                        .getShotAngularVelocity(Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS)),
                shooterCalculator.get().calculateShot().getIdealShot().getHoodAngle());
    }

    // no sudden ends
    @Override
    public void end(boolean interrupted) {
        transferInput = new TransferInput();

        transferSS.updateInputs(transferInput);
    }

    @Override
    public void execute() {

        shooterInput.setSpeed(-shooterSpeed.in(RPM));
        hoodInput.setRequestedPosition(hoodAngle.in(Degrees));
        turretInput.setRequestedPosition(turretAngle);

        shooterSS.updateInputs(shooterInput);
        hoodSS.updateInputs(hoodInput);
        turretSS.updateInputs(turretInput);

        boolean turretIsReady = turretSS.getOutputs().isAtRequestedPosition();
        boolean hoodIsReady = hoodSS.getOutputs().isAtRequestedPosition();
        boolean shooterIsReady = shooterSS.getOutputs().isAtSpeed();

        boolean isReadyToShoot = turretIsReady && hoodIsReady && shooterIsReady;

        if (isReadyToShoot) {
            transferInput.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
            transferSS.updateInputs(transferInput);
        } else {
            transferInput.setSpeed(0.0);
            transferSS.updateInputs(transferInput);
        }

        Logger.recordOutput("TurretIsReadyToShoot", turretIsReady + "");
        Logger.recordOutput("HoodIsReadyToShoot", hoodIsReady + "");
        Logger.recordOutput("ShooterIsReadyToShoot", shooterIsReady + "");
        Logger.recordOutput("IsReadyToShoot", isReadyToShoot + "");

    }

    @Override
    public void initialize() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
