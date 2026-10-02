package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;

/** Owns the transfer motors and disables transfer when either controller stops responding. */
public class TransferSubsystem extends EntechSubsystem<TransferInput, TransferOutput> {
    // Kept here because this change is limited to TransferSubsystem. 50 loops is about one second.
    private static final int MOTOR_CHECK_INTERVAL_LOOPS = 50;

    private double setSpeed = 0.0;
    private double setFlexSpeed = 0.0;
    private int loopsSinceMotorCheck = 0;
    private boolean motorsResponding = false;
    private boolean motorsConfigured = false;

    private SparkMax transferMotor;
    private SparkFlex transferFlexMotor;

    @Override
    public void initialize() {
        if (!RobotConstants.TRANSFER.ENABLED) {
            return;
        }

        transferMotor = new SparkMax(RobotConstants.PORTS.CAN.TRANSFER_MOTOR, MotorType.kBrushless);
        transferFlexMotor = new SparkFlex(RobotConstants.PORTS.CAN.TRANSFER_FLEX_MOTOR, MotorType.kBrushless);
        checkMotorCommunication();
    }

    /** Transfer is available only when requested by configuration and both motors respond. */
    @Override
    public boolean isEnabled() {
        return RobotConstants.TRANSFER.ENABLED && motorsResponding;
    }

    /** Applies motor settings once, retrying later if a controller was unavailable at startup. */
    private void configureMotors() throws MotorCommunicationException {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(RobotConstants.TRANSFER.BRAKING ? IdleMode.kBrake : IdleMode.kCoast);
        config.secondaryCurrentLimit(RobotConstants.TRANSFER.SECONDARY_CURRENT_LIMIT_AMPS);

        SparkFlexConfig flexConfig = new SparkFlexConfig();
        flexConfig.idleMode(RobotConstants.TRANSFER.BRAKING ? IdleMode.kBrake : IdleMode.kCoast);
        flexConfig.secondaryCurrentLimit(RobotConstants.TRANSFER.SECONDARY_CURRENT_LIMIT_AMPS);

        // Apply settings without repeated flash writes or persisting while the robot is enabled.
        requireSuccessfulResponse(transferMotor, transferMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        requireSuccessfulResponse(transferFlexMotor, transferFlexMotor.configure(
                flexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
        motorsConfigured = true;
    }

    /** Checks received motor data, rather than assuming sending a setpoint proves communication. */
    private void checkMotorCommunication() {
        try {
            checkMotorResponse(transferMotor);
            checkMotorResponse(transferFlexMotor);
            if (!motorsConfigured) {
                configureMotors();
            }
            motorsResponding = true;
            Logger.recordOutput("TransferSubsystem/MotorCommunicationError", "");
        } catch (MotorCommunicationException error) {
            disableTransfer();
            Logger.recordOutput("TransferSubsystem/MotorCommunicationError", error.getMessage());
        }
        Logger.recordOutput("TransferSubsystem/Enabled", isEnabled());
    }

    private void checkMotorResponse(SparkBase motor) throws MotorCommunicationException {
        motor.getBusVoltage();
        // REV errors are shared by devices on this thread, so read immediately after this motor's call.
        requireSuccessfulResponse(motor, motor.getLastError());
    }

    /** Converts an unsuccessful REV motor operation into the specific exception handled above. */
    private void requireSuccessfulResponse(SparkBase motor, REVLibError error) throws MotorCommunicationException {
        if (error != REVLibError.kOk) {
            throw new MotorCommunicationException("CAN ID " + motor.getDeviceId() + ": " + error);
        }
    }

    /** Stops any reachable motor once and discards the previous motion request. */
    private void disableTransfer() {
        if (motorsResponding) {
            transferMotor.stopMotor();
            transferFlexMotor.stopMotor();
        }
        motorsResponding = false;
        // A controller may have rebooted; reapply settings when communication returns.
        motorsConfigured = false;
        setSpeed = 0.0;
        setFlexSpeed = 0.0;
    }

    /** Applies and logs motor requests only while transfer is available. */
    @Override
    public void updateInputs(TransferInput input) {
        if (!isEnabled()) {
            return;
        }

        updateFlexSpeed(input);
        RobotIO.processInput(input);
        if (input.getSpeed() != setSpeed) {
            setSpeed = input.getSpeed();
            transferMotor.set(-setSpeed);
        }
        if (input.getFlexSpeed() != setFlexSpeed) {
            setFlexSpeed = input.getFlexSpeed();
            transferFlexMotor.set(-setFlexSpeed);
        }
    }

    /** Keeps the SparkFlex independently tunable while following the requested transfer direction. */
    private void updateFlexSpeed(TransferInput input) {
        double flexSpeed = LiveTuningHandler.getInstance().getValue(RobotConstants.TRANSFER.FLEX_SPEED_KEY);
        if (input.getSpeed() == 0.0 || !Double.isFinite(flexSpeed)) {
            input.setFlexSpeed(0.0);
        } else {
            input.setFlexSpeed(Math.signum(input.getSpeed()) * MathUtil.clamp(flexSpeed,
                    -RobotConstants.TRANSFER.MAX_OUTPUT, RobotConstants.TRANSFER.MAX_OUTPUT));
        }
    }

    /** Checks motors every N loops, including while unavailable, and refreshes live tuning. */
    @Override
    public void periodic() {
        if (!RobotConstants.TRANSFER.ENABLED || transferMotor == null || transferFlexMotor == null) {
            return;
        }

        loopsSinceMotorCheck++;
        if (loopsSinceMotorCheck >= MOTOR_CHECK_INTERVAL_LOOPS) {
            loopsSinceMotorCheck = 0;
            checkMotorCommunication();
        }

        if (isEnabled()) {
            TransferInput input = new TransferInput();
            input.setSpeed(setSpeed);
            updateInputs(input);
        }
    }

    @Override
    public Command getTestCommand() {
        return new TestTransferCommand(this);
    }

    @Override
    protected TransferOutput toOutputs() {
        TransferOutput output = new TransferOutput();
        output.setBraking(RobotConstants.TRANSFER.BRAKING);
        if (isEnabled()) {
            output.setTransferMotorOutput(SparkOutput.createOutput(transferMotor));
            output.setTransferFlexMotorOutput(SparkOutput.createOutput(transferFlexMotor));
        }
        return output;
    }

    /** REVLib returns error codes; this exception represents a failed required motor operation. */
    private static class MotorCommunicationException extends Exception {
        MotorCommunicationException(String message) {
            super(message);
        }
    }
}
