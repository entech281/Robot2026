package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;
import frc.robot.subsystems.turret.TurretOutput;

public class HoodSubsystem extends EntechSubsystem<HoodInput, HoodOutput> {

    private static final boolean ENABLED = true;

    private SparkMax hoodMotor;
    private SparkClosedLoopController hoodPIDController;
    private RelativeEncoder hoodEncoder;
    private HoodInput latestInput = new HoodInput();
    private SparkMaxConfig hoodConfig;


    @Override
    public void initialize() {
            if (!ENABLED) return;
            hoodMotor = new SparkMax(RobotConstants.PORTS.CAN.HOOD_MOTOR, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
            hoodConfig = new SparkMaxConfig();
            // Make encoder report degrees directly (adjust if your encoder reports rotations)
            hoodConfig.encoder.positionConversionFactor(RobotConstants.HOOD.POSITION_CONVERSION_FACTOR_DEGREES);
    
            // Closed-loop PIDF
            hoodConfig.closedLoop
                .pid(RobotConstants.HOOD.HOOD_P, RobotConstants.HOOD.HOOD_I, RobotConstants.HOOD.HOOD_D, ClosedLoopSlot.kSlot0);
    
            // Apply conservative signals update rates similar to other subsystems
            hoodConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0));
    
            // Configure the motor with these settings
            hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    
            hoodEncoder = hoodMotor.getEncoder();

            hoodPIDController = hoodMotor.getClosedLoopController();
            hoodPIDController.setSetpoint(hoodEncoder.getPosition(), ControlType.kPosition);
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(HoodInput input) {
        this.latestInput = input;
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    public HoodOutput toOutputs() {
        HoodOutput out = new HoodOutput();

        if (!ENABLED) return out;

        double currentPos = hoodEncoder.getPosition();
        double reqPos = latestInput.getRequestedPosition();

        // moving = whether position controller is actively trying to move (approx via velocity)
        out.setMoving(Math.abs(hoodEncoder.getVelocity()) > 1e-3);
        out.setRequestedPosition(reqPos);
        out.setCurrentPosition(currentPos);
        out.setAtForwardLimit(hoodMotor.getForwardLimitSwitch().isPressed());
        out.setAtReverseLimit(hoodMotor.getReverseLimitSwitch().isPressed());
        out.setAtRequestedPosition(Math.abs(currentPos - reqPos) <= RobotConstants.HOOD.HOOD_POSITION_TOLERANCE_DEGREES);

        out.setHoodMotor(SparkOutput.createOutput(hoodMotor));

        return out;
    }

    public void setHoodPosition(double desiredAngle) {
        if (!ENABLED) return;
        // clamp to allowed range
        double clamped = Math.max(RobotConstants.HOOD.HOOD_LOWER_LIMIT_DEGREES,
            Math.min(RobotConstants.HOOD.HOOD_UPPER_LIMIT_DEGREES, desiredAngle));
        latestInput.setRequestedPosition(clamped);

        if (hoodPIDController != null) {
            hoodPIDController.setSetpoint(clamped, ControlType.kPosition);
        } else {
            // if closed-loop is not ready, seed encoder
            hoodEncoder.setPosition(clamped);
        }
    }

    @Override
    public void periodic() {

        if (!ENABLED) return;

        double desiredPos = latestInput.getRequestedPosition();
        if (hoodPIDController != null) {
            setHoodPosition(desiredPos);
        }

    }
    
}
