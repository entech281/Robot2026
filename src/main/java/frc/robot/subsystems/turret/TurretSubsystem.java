/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
    private double desiredSpeed = 0.0;
    private TurretInput latestInput = new TurretInput();

    // private final ClampedDouble desiredTurretPositionEncoder = ClampedDouble.builder()
    //         .bounds(-5000000, 5000000)
    //         .withIncrement(10000.0)
    //         .withValue(0.0).build();

    @Override
    public void initialize() {
        if (!ENABLED) return;

        turretMotor = new SparkMax(RobotConstants.PORTS.CAN.TURRET_MOTOR, MotorType.kBrushless);
        // Basic configure - keep defaults for now. If you have a SparkMaxConfig, apply it here.
        turretMotor.getEncoder().setPosition(RobotConstants.TURRET.INITIAL_POSITION_DEGREES);

        reset();
    }

    public boolean isClockLimitHit() {
        if (!ENABLED) return false;
        return turretMotor.getForwardLimitSwitch().isPressed();
    }

    public void reset(){
        if (!ENABLED) return;
        turretMotor.set(0);
        desiredSpeed = 0.0;
    }

    public void turnTurret(Double speed){
        if (!ENABLED) return;
        desiredSpeed = Math.max(-1.0, Math.min(1.0, speed));
        turretMotor.set(desiredSpeed);
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
        turnTurret(-0.4);
    }

    private void adjustTurretClockwisePercent() {
        turnTurret(0.4);
    }

    // position control not implemented in this simplified turret. Use percent output via turnTurret().

    public void setTurretPosition(double desiredAngle) {
        // For now, just set the encoder to requested position (no closed-loop control)
        if (!ENABLED) return;
        turretMotor.getEncoder().setPosition(desiredAngle);
    }

    @Override
    public void periodic() {
        if (!ENABLED) return;
        // keep commanded speed applied
        this.turnTurret(latestInput.getRequestedSpeed());
        // Basic logging removed (no shared logger instance here). Use outputs for dashboard/logging.
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

        out.setMoving(Math.abs(desiredSpeed) > 1e-4);
        out.setRequestedPosition(latestInput.getRequestedPosition());
        out.setCurrentPosition(turretMotor.getEncoder().getPosition());
        out.setAtBlueLimit(turretMotor.getForwardLimitSwitch().isPressed());
        out.setAtRedLimit(turretMotor.getReverseLimitSwitch().isPressed());
        out.setAtRequestedSpeed(Math.abs(desiredSpeed - latestInput.getRequestedSpeed()) <= 0.001);

        out.setTurretMotor(SparkMaxOutput.createOutput(turretMotor));

        return out;
    }

}
