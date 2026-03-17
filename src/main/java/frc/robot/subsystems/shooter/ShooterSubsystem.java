package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import java.util.Arrays;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;

public class ShooterSubsystem extends EntechSubsystem<ShooterInput, ShooterOutput> {
    private boolean ENABLED = true;
    private SparkFlex shooterMotorA;
    private SparkFlex shooterMotorB;

    private double setSpeed = 0.0;

    private double[] lastLiveTuning = new double[6];

    private static final boolean BRAKING = false;

    @Override
    public void initialize() {
        if (ENABLED) {
            lastLiveTuning = grabLiveTuning();
            shooterMotorA = new SparkFlex(RobotConstants.PORTS.CAN.SHOOTER_MOTOR_A, SparkFlex.MotorType.kBrushless);
            shooterMotorB = new SparkFlex(RobotConstants.PORTS.CAN.SHOOTER_MOTOR_B, SparkFlex.MotorType.kBrushless);

            configure(lastLiveTuning);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(ShooterInput input) {
        RobotIO.processInput(input);
        if (ENABLED) {
            setSpeed = input.getSpeed();
            if (setSpeed == 0.0) {
                shooterMotorA.set(0);
            } else {
                shooterMotorA.getClosedLoopController().setSetpoint(input.getSpeed(),
                        ControlType.kVelocity,
                        ClosedLoopSlot.kSlot0);
            }
        }
    }

    @Override
    public Command getTestCommand() {
        return new TestShooterCommand(this);
    }

    @Override
    protected ShooterOutput toOutputs() {
        ShooterOutput so = new ShooterOutput();

        if (ENABLED) {
            so.setSpeed(setSpeed);
            so.setBraking(BRAKING);
            so.setAtSpeed(Math.abs(
                    shooterMotorA.getEncoder().getVelocity() - setSpeed) <= RobotConstants.SHOOTER.TOLERANCE.in(RPM));

            so.setShooterMotorA(SparkOutput.createOutput(shooterMotorA));
            so.setShooterMotorB(SparkOutput.createOutput(shooterMotorB));
        }

        return so;
    }

    @Override
    public void periodic() {
        double[] live = grabLiveTuning();
        if (!Arrays.equals(live, lastLiveTuning)) {
            configure(live);
            lastLiveTuning = live;
        }
    }

    private void configure(double[] factors) {
        SparkFlexConfig shooterAConfig = new SparkFlexConfig();

        shooterAConfig.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);
        shooterAConfig.smartCurrentLimit(160);
        shooterAConfig.closedLoop.feedForward.kV(factors[3]);
        shooterAConfig.closedLoop.feedForward.kA(factors[4]);
        shooterAConfig.closedLoop.feedForward.kS(factors[5]);
        shooterAConfig.encoder.velocityConversionFactor(1.0);
        shooterAConfig.closedLoop.pid(factors[0], factors[1], factors[2], ClosedLoopSlot.kSlot0);
        shooterAConfig.voltageCompensation(12.5);
        shooterAConfig.inverted(false);

        shooterMotorA.configure(shooterAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig shooterBConfig = new SparkFlexConfig().apply(shooterAConfig);
        shooterBConfig.follow(shooterMotorA, true);

        shooterMotorB.configure(shooterBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private double[] grabLiveTuning() {
        return new double[] {
                LiveTuningHandler.getInstance().getValue("ShooterSubsystem/kP"),
                LiveTuningHandler.getInstance().getValue("ShooterSubsystem/kI"),
                LiveTuningHandler.getInstance().getValue("ShooterSubsystem/kD"),
                LiveTuningHandler.getInstance().getValue("ShooterSubsystem/kV"),
                LiveTuningHandler.getInstance().getValue("ShooterSubsystem/kA"),
                LiveTuningHandler.getInstance().getValue("ShooterSubsystem/kS")
        };
    }
}
