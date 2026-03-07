package frc.robot.subsystems.hopper;

import java.lang.module.ModuleReader;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.entech.util.stall.MotorStallDetector;
import frc.robot.RobotConstants;
import frc.robot.commands.DeployHopper;

public class HopperSubsystem extends EntechSubsystem<HopperInput, HopperOutput> {
    private static final boolean ENABLED = false;
    private static final boolean BRAKING = false; // Set to false for compliance (coast mode) to allow movement if hit

    private SparkFlex hopperMotor;
    private SparkLimitSwitch lowerLimitSwitch;
    private double setSpeed = 0.0;
    private MotorStallDetector stallDetector;

    @Override
    public void initialize() {
        if (ENABLED) {
            hopperMotor = new SparkFlex(RobotConstants.PORTS.CAN.HOPPER_MOTOR, MotorType.kBrushless);

            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);

            LimitSwitchConfig limitConfig = new LimitSwitchConfig();
            limitConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
            config.apply(limitConfig);

            hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            lowerLimitSwitch = hopperMotor.getForwardLimitSwitch();

            stallDetector = MotorStallDetector.Builder.defaults();
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(HopperInput input) {
        if (ENABLED) {
            if (setSpeed != input.getSpeed()) {
                boolean isStalled = stallDetector != null && stallDetector.isStalled(hopperMotor);

                if (isStalled && input.getSpeed() > 0) {
                    hopperMotor.set(0.0);
                    setSpeed = 0.0;
                } else {
                    hopperMotor.set(input.getSpeed());
                    setSpeed = input.getSpeed();
                }
            }
        }
    }

    @Override
    public Command getTestCommand() {
        return new DeployHopper(this);
    }

    @Override
    protected HopperOutput toOutputs() {
        HopperOutput output = new HopperOutput();

        if (ENABLED) {
            output.setHopperMotorOutput(SparkOutput.createOutput(hopperMotor));
            output.setAtLowerLimit(lowerLimitSwitch.isPressed());
            output.setStalled(stallDetector != null && stallDetector.isStalled(hopperMotor));
        }

        return output;
    }

    @Override
    public void periodic() {
        if (!ENABLED) {
            return;
        }

        if (lowerLimitSwitch.isPressed()) {
            HopperInput newInput = new HopperInput();
            newInput.setSpeed(0.0);
            updateInputs(newInput);
        }
    }

}