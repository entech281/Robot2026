package frc.robot.subsystems.prototype2;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;

public class PrototypeSubsystem2 extends EntechSubsystem<PrototypeInput2, PrototypeOutput2> {
    private static final boolean ENABLED = true;
    private SparkFlex motor;
    @Override
    public void initialize() {
        if (ENABLED){
            motor = new SparkFlex(10293, MotorType.kBrushless);

            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(IdleMode.kCoast);
            config.voltageCompensation(12.5);
            config.smartCurrentLimit(40);
            config.secondaryCurrentLimit(60);

            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }

    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(PrototypeInput2 input) {
        if (ENABLED){
            motor.set(input.getSpeed());
        }
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected PrototypeOutput2 toOutputs() {
        PrototypeOutput2 po = new PrototypeOutput2();
        if (ENABLED){
            po.setSparkOutput(SparkOutput.createOutput(motor));
        }
        return po;
    }
    
}
