package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;

public class HopperSubsystem extends EntechSubsystem<HopperInput, HopperOutput> {
    private static final boolean ENABLED = true;
    private static final boolean BRAKING = false; 

    private SparkMax hopperMotor;
    private double setSpeed = 0.0;

    @Override
    public void initialize() {
        if (ENABLED) {
            hopperMotor = new SparkMax(RobotConstants.PORTS.CAN.HOPPER_MOTOR, MotorType.kBrushless);

            SparkMaxConfig config = new SparkMaxConfig();
            config.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);

            hopperMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
                hopperMotor.set(input.getSpeed());
                setSpeed = input.getSpeed();
            }
        }
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected HopperOutput toOutputs() {
        HopperOutput output = new HopperOutput();

        if (ENABLED) {
            output.setHopperMotorOutput(SparkOutput.createOutput(hopperMotor));
        }

        return output;
    }

}