/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkMaxOutput;
import frc.robot.RobotConstants;

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

    // private final ClampedDouble desiredTurretPositionEncoder = ClampedDouble.builder()
    //         .bounds(-5000000, 5000000)
    //         .withIncrement(10000.0)
    //         .withValue(0.0).build();

    @Override
    public void initialize() {
        if (!ENABLED) return;
        turretMotor = new SparkMax(RobotConstants.PORTS.CAN.TURRET_MOTOR, MotorType.kBrushless);

        turretConfig = new SparkMaxConfig();
        // Make encoder report degrees directly (adjust if your encoder reports rotations)
        turretConfig.encoder.positionConversionFactor(RobotConstants.TURRET.POSITION_CONVERSION_FACTOR_DEGREES);

        // Closed-loop PIDF
        turretConfig.closedLoop
            .pid(RobotConstants.TURRET.TURRET_POSITION_P, RobotConstants.TURRET.TURRET_POSITION_I,
                RobotConstants.TURRET.TURRET_POSITION_D, ClosedLoopSlot.kSlot0)
            .feedForward.kV(RobotConstants.TURRET.TURRET_POSITION_FF, ClosedLoopSlot.kSlot0);

        // Apply conservative signals update rates similar to other subsystems
        turretConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0));

        // Configure the motor with these settings
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turretEncoder = turretMotor.getEncoder();
        turretEncoder.setPosition(RobotConstants.TURRET.INITIAL_POSITION_DEGREES);

        turretPIDController = turretMotor.getClosedLoopController();

        // seed desired position to current
        turretPIDController.setSetpoint(turretEncoder.getPosition(), ControlType.kPosition);

        reset();
    }

    public boolean isClockLimitHit() {
        if (!ENABLED) return false;
        return turretMotor.getForwardLimitSwitch().isPressed();
    }

    public void reset(){
        if (!ENABLED) return;
        // stop motor and reset desired speed and requests
        turretMotor.set(0);
        latestInput.setRequestedPosition(turretEncoder.getPosition());
        // set closed-loop setpoint to current position
        if (turretPIDController != null) {
            turretPIDController.setSetpoint(turretEncoder.getPosition(), ControlType.kPosition);
        }
    }


    public boolean isCounterClockLimitHit() {
        if (!ENABLED) return false;
        return turretMotor.getReverseLimitSwitch().isPressed();
    }


    public void adjustTurretCounterClockwise() {
        // simple percent output based adjust
        adjustTurretCounterClockwisePercent();
    }

    public void adjustTurretClockwise() {
        // simple percent output based adjust
        adjustTurretClockwisePercent();
    }

    private void adjustTurretCounterClockwisePercent() {
        // incremental position change
        setTurretPosition(turretEncoder.getPosition() - RobotConstants.TURRET.TURRET_ADJUST_STEP_DEGREES);
    }

    private void adjustTurretClockwisePercent() {
        // incremental position change
        setTurretPosition(turretEncoder.getPosition() + RobotConstants.TURRET.TURRET_ADJUST_STEP_DEGREES);
    }

    // position control not implemented in this simplified turret. Use percent output via turnTurret().

    public void setTurretPosition(double desiredAngle) {
        if (!ENABLED) return;
        // clamp to allowed range
        double clamped = Math.max(RobotConstants.TURRET.MIN_TURRET_ANGLE_DEGREES,
            Math.min(RobotConstants.TURRET.MAX_TURRET_ANGLE_DEGREES, desiredAngle));
        latestInput.setRequestedPosition(clamped);
        if (turretPIDController != null) {
            turretPIDController.setSetpoint(clamped, ControlType.kPosition);
        } else {
            // if closed-loop is not ready, seed encoder
            turretEncoder.setPosition(clamped);
        }
    }

    @Override
    public void periodic() {
        if (!ENABLED) return;
        // Position control: continuously ensure setpoint equals latest requested position
        double desiredPos = latestInput.getRequestedPosition();
        if (turretPIDController != null) {
            turretPIDController.setSetpoint(desiredPos, ControlType.kPosition);
        }
    }
    

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(TurretInput input) {
        // store latest requested position from input for toOutputs
        this.latestInput = input;
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    public TurretOutput toOutputs() {
        TurretOutput out = new TurretOutput();

        if (!ENABLED) return out;

        double currentPos = turretEncoder.getPosition();
        double reqPos = latestInput.getRequestedPosition();

    // moving = whether position controller is actively trying to move (approx via velocity)
    out.setMoving(Math.abs(turretEncoder.getVelocity()) > 1e-3);
        out.setRequestedPosition(reqPos);
        out.setCurrentPosition(currentPos);
        out.setAtBlueLimit(turretMotor.getForwardLimitSwitch().isPressed());
        out.setAtRedLimit(turretMotor.getReverseLimitSwitch().isPressed());
    out.setAtRequestedPosition(Math.abs(currentPos - reqPos) <= RobotConstants.TURRET.TURRET_POSITION_TOLERANCE_DEGREES);

        out.setTurretMotor(SparkMaxOutput.createOutput(turretMotor));

        return out;
    }

}
