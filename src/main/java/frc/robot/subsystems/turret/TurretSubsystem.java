/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.entech.util.stall.MotorStallDetector;
import frc.robot.RobotConstants;
import frc.robot.livetuning.LiveTuningHandler;

/**
 *
 * @author aryan - for hoodSubsystem
 * @modifiedBy rohit for turretSubsytem
 */
public class TurretSubsystem extends EntechSubsystem<TurretInput, TurretOutput> {

    private static final boolean ENABLED = true;

    private SparkMax turretMotor;
    private SparkClosedLoopController turretPIDController;
    private RelativeEncoder turretEncoder;
    private TurretInput latestInput = new TurretInput();
    private SparkMaxConfig turretConfig;
    private MotorStallDetector stallDetector;

    @Override
    public void initialize() {
        if (!ENABLED)
            return;
        turretMotor = new SparkMax(RobotConstants.PORTS.CAN.TURRET_MOTOR, MotorType.kBrushless);

        turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(IdleMode.kBrake);
        // Make encoder report degrees directly (adjust if your encoder reports
        // rotations)
        turretConfig.encoder.positionConversionFactor(RobotConstants.TURRET.POSITION_CONVERSION_FACTOR_DEGREES);

        // turretConfig.closedLoop
        //     .maxMotion
        //     .cruiseVelocity(RobotConstants.TURRET.TURRET_CRUISE_VELOCITY_RPM, ClosedLoopSlot.kSlot0)
        //     .maxAcceleration(RobotConstants.TURRET.TURRET_MAX_ACCELERATION_RPM_PER_SECOND, ClosedLoopSlot.kSlot0)
        //     .allowedProfileError(RobotConstants.TURRET.TURRET_ALLOWED_PROFILE_ERROR_ROTATIONS, ClosedLoopSlot.kSlot0);

                // Closed-loop PIDF
        turretConfig.closedLoop
                .pid(RobotConstants.TURRET.TURRET_POSITION_P, RobotConstants.TURRET.TURRET_POSITION_I,
                        RobotConstants.TURRET.TURRET_POSITION_D, ClosedLoopSlot.kSlot0).feedForward
                .kV(RobotConstants.TURRET.TURRET_POSITION_FF, ClosedLoopSlot.kSlot0);

        // Apply conservative signals update rates similar to other subsystems
        turretConfig.signals
                .primaryEncoderPositionAlwaysOn(true);

        // Configure the motor with these settings
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turretEncoder = turretMotor.getEncoder();

        turretPIDController = turretMotor.getClosedLoopController();
        // create a persistent stall detector once
        stallDetector = MotorStallDetector.Builder.defaults();

        // seed desired position to current
        turretPIDController.setSetpoint(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        reset();
    }

    public void reset() {
        if (!ENABLED)
            return;
        // stop motor and reset desired speed and requests
        turretMotor.set(0);
        turretEncoder.setPosition(RobotConstants.TURRET.HOME_POSITION_DEGREES);
        latestInput.setRequestedPosition(0.0);
        // set closed-loop setpoint to current position
        turretPIDController.setSetpoint(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    private void setTurretPosition(double desiredAngle) {
        if (!ENABLED)
            return;

        // clamp to allowed range
        double clamped = Math.max(LiveTuningHandler.getInstance().getValue("TurretSubsystem/LowerLimitDegrees"),
                Math.min(LiveTuningHandler.getInstance().getValue("TurretSubsystem/UpperLimitDegrees"), desiredAngle));

        boolean isStalled = (stallDetector != null && stallDetector.isStalled(turretMotor));
        if (isStalled && turretEncoder.getPosition() < 0) {
            if (clamped < turretEncoder.getPosition()) {
                clamped = turretEncoder.getPosition();
            }
        } else if (isStalled && turretEncoder.getPosition() > 0) {
            if (clamped > turretEncoder.getPosition()) {
                clamped = turretEncoder.getPosition();
            }
        }

        turretPIDController.setSetpoint(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);        
    }

    @Override
    public void periodic() {
        if (!ENABLED)
            return;
        double desiredPos = latestInput.getRequestedPosition();
        setTurretPosition(desiredPos);
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(TurretInput input) {
        this.latestInput = input;
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    public TurretOutput toOutputs() {
        TurretOutput out = new TurretOutput();

        if (!ENABLED)
            return out;

        double currentPos = turretEncoder.getPosition();
        double reqPos = latestInput.getRequestedPosition();

        boolean isStalled = (stallDetector != null && stallDetector.isStalled(turretMotor));

        out.setIsStalled(isStalled);

        if (isStalled && turretEncoder.getPosition() < 0) {
            out.setAtReverseLimitStall(true);
            out.setAtForwardLimitStall(false);
        } else if (isStalled && turretEncoder.getPosition() > 0) {
            out.setAtReverseLimitStall(false);
            out.setAtForwardLimitStall(true);
        } else {
            out.setAtReverseLimitStall(false);
            out.setAtForwardLimitStall(false);
        }

        // TODO: soft limit utility class

        if (turretEncoder.getPosition() < LiveTuningHandler.getInstance()
                .getValue("TurretSubsystem/SofterLowerLimitDegrees")) {
            out.setPastSofterLowerLimit(true);
        } else {
            out.setPastSofterLowerLimit(false);
        }

        if (turretEncoder.getPosition() > LiveTuningHandler.getInstance()
                .getValue("TurretSubsystem/SofterUpperLimitDegrees")) {
            out.setPastSofterUpperLimit(true);
        } else {
            out.setPastSofterUpperLimit(false);
        }

        // moving = whether position controller is actively trying to move (approx via
        // velocity)
        out.setMoving(Math.abs(turretEncoder.getVelocity()) > 1e-3);
        out.setRequestedPosition(reqPos);
        out.setCurrentPosition(currentPos);
        out.setAtRequestedPosition(
                Math.abs(currentPos - reqPos) <= RobotConstants.TURRET.TURRET_POSITION_TOLERANCE_DEGREES);
            out.setTurretMotor(SparkOutput.createOutput(turretMotor));

        return out;
    }

}
