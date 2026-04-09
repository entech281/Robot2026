package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;

public class HoodSubsystem extends EntechSubsystem<HoodInput, HoodOutput> {
    private static final boolean ENABLED = false;

    private SparkMax hoodMotor;
    private SparkClosedLoopController hoodPIDController;
    private RelativeEncoder hoodEncoder;
    private HoodInput latestInput = new HoodInput();
    private SparkMaxConfig hoodConfig;

    @Override
    public void initialize() {
        if (!ENABLED)
            return;
        hoodMotor = new SparkMax(RobotConstants.PORTS.CAN.HOOD_MOTOR,
                com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        hoodConfig = new SparkMaxConfig();
        // Make encoder report degrees directly (adjust if your encoder reports
        // rotations)
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.inverted(false);
        hoodConfig.encoder.positionConversionFactor(RobotConstants.HOOD.POSITION_CONVERSION_FACTOR_DEGREES);
        hoodConfig.encoder
                .velocityConversionFactor(RobotConstants.HOOD.VELOCITY_CONVERSION_FACTOR_DEGREES_PER_SECOND_PER_RPM);

        // Closed-loop PIDF
        hoodConfig.closedLoop
                .pid(RobotConstants.HOOD.HOOD_P, RobotConstants.HOOD.HOOD_I, RobotConstants.HOOD.HOOD_D,
                        ClosedLoopSlot.kSlot0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        hoodConfig.closedLoop.maxMotion
                .cruiseVelocity(RobotConstants.HOOD.HOOD_CRUISE_VELOCITY.in(DegreesPerSecond))
                .maxAcceleration(RobotConstants.HOOD.HOOD_MAX_ACCELERATION.in(DegreesPerSecondPerSecond))
                .allowedProfileError(RobotConstants.HOOD.HOOD_ALLOWED_PROFILE_ERROR_ROTATIONS);

        // Apply conservative signals update rates similar to other subsystems
        hoodConfig.signals
                .primaryEncoderPositionAlwaysOn(true);

        hoodConfig.limitSwitch.reverseLimitSwitchPosition(0.0)
                .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        hoodConfig.smartCurrentLimit(5);

        // Configure the motor with these settings
        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        RobotIO.processInput(input);
        this.latestInput = input;
    }

    @Override
    public Command getTestCommand() {
        return new TestHoodCommand(this);
    }

    @Override
    public HoodOutput toOutputs() {
        HoodOutput out = new HoodOutput();

        if (!ENABLED) {
            out.setAtRequestedPosition(true);
            return out;
        }

        double reqPos = latestInput.getRequestedPosition();
        SparkOutput spark = SparkOutput.createOutput(hoodMotor);

        // moving = whether position controller is actively trying to move (approx via
        // velocity)
        out.setMoving(Math.abs(spark.getCurrentSpeed()) > 0.0001);
        out.setRequestedPosition(reqPos);
        out.setAtRequestedPosition(
                Math.abs(spark.getCurrentPosition() - reqPos) <= RobotConstants.HOOD.HOOD_POSITION_TOLERANCE_DEGREES);

        out.setHoodMotor(spark);

        out.setCurrentPosition(spark.getCurrentPosition());

        return out;
    }

    public void setHoodPosition(double desiredAngle) {
        if (!ENABLED)
            return;
        // clamp to allowed range
        double clamped = Math.max(RobotConstants.HOOD.HOOD_LOWER_LIMIT_DEGREES,
                Math.min(RobotConstants.HOOD.HOOD_UPPER_LIMIT_DEGREES, desiredAngle));
        latestInput.setRequestedPosition(clamped);

        if (Math.abs(hoodEncoder.getPosition()
                - latestInput.getRequestedPosition()) > RobotConstants.HOOD.HOOD_POSITION_TOLERANCE_DEGREES) {
            hoodPIDController.setSetpoint(desiredAngle, ControlType.kMAXMotionPositionControl);
        } else {
            hoodMotor.set(0.0);
        }
    }

    @Override
    public void periodic() {
        if (!ENABLED)
            return;

        if (hoodMotor.getReverseLimitSwitch().isPressed() && latestInput.getRequestedPosition() == 0.0) {
            hoodEncoder.setPosition(0);
        }
        double desiredPos = latestInput.getRequestedPosition();
        if (hoodPIDController != null) {
            setHoodPosition(desiredPos);
        }
    }
}
