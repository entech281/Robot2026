package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.entech.commands.EntechCommand;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.subsystems.hood.HoodInput;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterInput;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferInput;
import frc.robot.subsystems.transfer.TransferSubsystem;
import frc.robot.subsystems.turret.TurretInput;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.AimingCalculator;
import frc.robot.util.AimingOutputData;

public class VirtualTargetAutoShootCommand extends EntechCommand {
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final TransferSubsystem transfer;
    private final TurretSubsystem turret;
    private boolean speedReached = false;
    private boolean snowblow;
    private Supplier<Pose3d> snowblowSupplier;
    TurretInput tui = new TurretInput();
    ShooterInput si = new ShooterInput();
    TransferInput tri = new TransferInput();
    HoodInput hi = new HoodInput();

    public VirtualTargetAutoShootCommand(ShooterSubsystem shooter, HoodSubsystem hood, TransferSubsystem transfer,
            TurretSubsystem turret, boolean snowblow, Supplier<Pose3d> snowblowSupplier) {
        super(shooter, hood, transfer, turret);
        this.shooter = shooter;
        this.hood = hood;
        this.transfer = transfer;
        this.turret = turret;
        this.snowblow = snowblow;
        this.snowblowSupplier = snowblowSupplier;
    }

    public VirtualTargetAutoShootCommand(ShooterSubsystem shooter, HoodSubsystem hood, TransferSubsystem transfer,
            TurretSubsystem turret) {
        this(shooter, hood, transfer, turret, false, () -> new Pose3d());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        hood.acceptInputs(new HoodInput());
        transfer.acceptInputs(new TransferInput());

        speedReached = false;
    }

    @Override
    public void execute() {
        Pose3d robotPose = new Pose3d(RobotIO.getInstance().getOdometryPose());

        Pose3d targetPose;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!snowblow) {
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                targetPose = RobotConstants.TURRET.RED_HUB_LOCATION;
            } else {
                targetPose = RobotConstants.TURRET.BLUE_HUB_LOCATION;
            }
        } else {
            targetPose = snowblowSupplier.get();
        }

        AimingOutputData shotData = AimingCalculator.calculateAimingData(robotPose, targetPose,
                RobotIO.getInstance().getDriveOutput().getSpeeds());

        hi.setRequestedPosition(shotData.getHoodAngle().in(Degrees));
        si.setSpeed(shotData.getShooterSpeed().in(RPM));
        tui.setRequestedPosition(shotData.getTurretAngle());

        turret.acceptInputs(tui);
        shooter.acceptInputs(si);
        hood.acceptInputs(hi);

        if (!speedReached) {
            speedReached = shooter.getOutputs().isAtSpeed();
        }
        Logger.recordOutput("target", targetPose);
        if (RobotIO.getInstance().getHoodOutput().isAtRequestedPosition() && speedReached || snowblow) {
            tri.setSpeed(LiveTuningHandler.getInstance().getValue("TransferSubsystem/SetSpeed"));
        }

        transfer.acceptInputs(tri);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
