package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkMaxOutput;
import frc.robot.RobotConstants;

public class ShooterSubsystem extends EntechSubsystem<ShooterInput, ShooterOutput> {
    private boolean ENABLED = true;
    private SparkFlex shooterMotorA;
    private SparkFlex shooterMotorB;

    private double setSpeed = 0.0;

    private static final boolean BRAKING = false;

    @Override
    public void initialize() {
        if (ENABLED) {
            shooterMotorA = new SparkFlex(RobotConstants.PORTS.CAN.SHOOTER_MOTOR_A, SparkFlex.MotorType.kBrushless);
            shooterMotorB = new SparkFlex(RobotConstants.PORTS.CAN.SHOOTER_MOTOR_B, SparkFlex.MotorType.kBrushless);

            SparkFlexConfig shooterAConfig = new SparkFlexConfig();

            shooterAConfig.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);
            // shooterAConfig.smartCurrentLimit(40);
            // shooterConfig.closedLoop.feedForward.;
            shooterAConfig.closedLoop.pid(0.001, 0.0, 0.0, ClosedLoopSlot.kSlot0);

            shooterMotorA.configure(shooterAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            SparkFlexConfig shooterBConfig = new SparkFlexConfig().apply(shooterAConfig);
            shooterBConfig.follow(shooterMotorA, true);

            shooterMotorB.configure(shooterBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(ShooterInput input) {
        if (ENABLED) {
            if (setSpeed != input.getSpeed()) {
                shooterMotorA.getClosedLoopController().setSetpoint(input.getSpeed(), ControlType.kVelocity,
                        ClosedLoopSlot.kSlot0);
                setSpeed = input.getSpeed();
                shooterMotorA.set(setSpeed);
            }
        }
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected ShooterOutput toOutputs() {
        ShooterOutput so = new ShooterOutput();
        so.setSpeed(setSpeed);
        so.setBraking(BRAKING);
        so.setAtSpeed(shooterMotorA.getClosedLoopController().isAtSetpoint());

        so.setShooterMotorA(SparkMaxOutput.createOutput(shooterMotorA));
        so.setShooterMotorB(SparkMaxOutput.createOutput(shooterMotorB));
        return so;
    }
}
