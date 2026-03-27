/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.entech.util.EntechUtils;
import frc.entech.util.stall.MotorStallDetector;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;

/**
 *
 * @author aryan - for hoodSubsystem
 * @modifiedBy rohit for turretSubsytem
 */
public class TurretSubsystem extends EntechSubsystem<TurretInput, TurretOutput> {

    private static final boolean ENABLED = true;
    private boolean homed = false;

    private SparkMax turretMotor;
    private TurretEncoder turretEncoder;
    private TurretInput latestInput = new TurretInput();
    private SparkMaxConfig turretConfig;
    private MotorStallDetector stallDetector;
    private double PID_MAX = 1;
    private double PID_MIN = -1;
    private DigitalInput reverseLimitSwitch;
    private DigitalInput forwardLimitSwitch;
    private boolean inverted = false;

    private boolean lastLimitSwitchState = false;

    private PIDController control = new PIDController(RobotConstants.TURRET.TURRET_POSITION_P,
            RobotConstants.TURRET.TURRET_POSITION_I,
            RobotConstants.TURRET.TURRET_POSITION_D);

    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            RobotConstants.TURRET.TURRET_CRUISE_VELOCITY.in(DegreesPerSecond),
            RobotConstants.TURRET.TURRET_MAX_ACCELERATION.in(DegreesPerSecondPerSecond)));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    @Override
    public void initialize() {
        if (!ENABLED)
            return;
        reverseLimitSwitch = new DigitalInput(RobotConstants.PORTS.DIO.HOME_TURRET_SWITCH);
        forwardLimitSwitch = new DigitalInput(RobotConstants.PORTS.DIO.FORWARD_TURRET_SWITCH);

        turretMotor = new SparkMax(RobotConstants.PORTS.CAN.TURRET_MOTOR, MotorType.kBrushless);
        turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(IdleMode.kBrake);
        // Make encoder report degrees directly
        turretConfig.encoder
                .positionConversionFactor(RobotConstants.TURRET.POSITION_CONVERSION_FACTOR_INTERNAL_ENCODER);

        turretEncoder = new TurretEncoder(turretMotor.getAbsoluteEncoder());

        turretConfig.inverted(true);

        turretEncoder.setPositionConversionFactor(RobotConstants.TURRET.POSITION_CONVERSION_FACTOR_ABSOLUTE_ENCODER);

        // Configure the motor with these settings
        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // create a persistent stall detector once
        stallDetector = MotorStallDetector.Builder.defaults();

        reset();
    }

    public void reset() {
        if (!ENABLED)
            return;
        // stop motor and reset desired speed and requests
        turretMotor.set(0);
        turretEncoder.setPosition(RobotConstants.TURRET.HOME_POSITION_DEGREES.in(Degrees));
        turretMotor.getEncoder().setPosition(RobotConstants.TURRET.HOME_POSITION_DEGREES.in(Degrees));
        latestInput.setRequestedPosition(Degrees.of(0.0));
    }

    // TODO: move to EntechUtils and Unit test
    private double circularDistance(double a, double b) {
        double diff = Math.abs(a - b);
        return Math.min(diff, 360 - diff);
    }

    private void setTurretPosition(Angle desiredAngle) {
        if (!ENABLED)
            return;

        double lowerLimit = LiveTuningHandler.getInstance().getValue("TurretSubsystem/LowerLimitDegrees");
        double upperLimit = LiveTuningHandler.getInstance().getValue("TurretSubsystem/UpperLimitDegrees");

        double clamped = desiredAngle.in(Degrees);

        if (desiredAngle.in(Degrees) > upperLimit || desiredAngle.in(Degrees) < lowerLimit) {
            double distToLower = circularDistance(desiredAngle.in(Degrees), lowerLimit);
            double distToUpper = circularDistance(desiredAngle.in(Degrees), upperLimit);

            clamped = (distToLower < distToUpper) ? lowerLimit : upperLimit;
        }

        if (inverted) {
            clamped = -clamped;
        }

        // boolean isStalled = (stallDetector != null &&
        // stallDetector.isStalled(turretMotor));
        // if (isStalled && turretEncoder.getPosition() < 0) {
        // if (clamped < turretEncoder.getPosition()) {
        // clamped = turretEncoder.getPosition();
        // }
        // } else if (isStalled && turretEncoder.getPosition() > 0) {
        // if (clamped > turretEncoder.getPosition()) {
        // clamped = turretEncoder.getPosition();
        // }
        // }

        m_goal = new TrapezoidProfile.State(clamped, 0);
        m_setpoint = m_profile.calculate(RobotConstants.TURRET.TRAPEZOIDAL_DELTA_TIME.in(Seconds), m_setpoint, m_goal);

        if (m_setpoint.position > turretEncoder.getPosition() && getForwardLimitSwitch()) {
            turretMotor.set(0.0);
        } else {
            turretMotor
                    .set(EntechUtils.capDoubleValue(control.calculate(turretEncoder.getPosition(), m_setpoint.position),
                            PID_MIN, PID_MAX));
        }
    }

    @Override
    public void periodic() {
        if (!ENABLED)
            return;
        Angle desiredPos = latestInput.getRequestedPosition();
        if (getReverseLimitSwitch() && getReverseLimitSwitch() != lastLimitSwitchState && !homed) {
            homed = true;
            turretEncoder.setPosition(LiveTuningHandler.getInstance().getValue("TurretSubsystem/HomeSwitchPosition"));
            turretMotor.getEncoder()
                    .setPosition(LiveTuningHandler.getInstance().getValue("TurretSubsystem/HomeSwitchPosition"));
        }

        lastLimitSwitchState = getReverseLimitSwitch();
        if (latestInput.getActivate() && homed) {
            setTurretPosition(desiredPos);
        } else {
            turretMotor.set(0.0);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(TurretInput input) {
        RobotIO.processInput(input);
        this.latestInput = input;
    }

    @Override
    public Command getTestCommand() {
        return new TestTurretCommand(this);
    }

    @Override
    public TurretOutput toOutputs() {
        TurretOutput out = new TurretOutput();

        if (!ENABLED)
            return out;

        Angle currentPos = Degrees.of(turretEncoder.getPosition());
        Angle reqPos = latestInput.getRequestedPosition();

        boolean isStalled = (stallDetector != null && stallDetector.isStalled(turretMotor));

        out.setIsStalled(isStalled);
        out.setHomed(homed);
        out.setHomingSwitchState(getReverseLimitSwitch());
        out.setFarLimitSwitchState(getForwardLimitSwitch());

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
        out.setCurrentPosition(inverted ? currentPos.times(-1) : currentPos);
        out.setAtRequestedPosition(
                Math.abs(currentPos.minus(reqPos)
                        .in(Degrees)) <= RobotConstants.TURRET.TURRET_POSITION_TOLERANCE_DEGREES.in(Degrees));
        out.setTurretMotor(SparkOutput.createOutput(turretMotor));

        return out;
    }

    private boolean getReverseLimitSwitch() {
        return reverseLimitSwitch.get();
    }

    private boolean getForwardLimitSwitch() {
        return forwardLimitSwitch.get();
    }
}
