package frc.robot.subsystems.intake;

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
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;

public class IntakeSubsystem extends EntechSubsystem<IntakeInput, IntakeOutput> {
    private static final boolean ENABLED = true;
    private static final boolean BRAKING = true;
    private SparkFlex intakeMotor;
    private double setSpeed = 0.0;

    @Override
    public void initialize() {
        if (ENABLED) {
            intakeMotor = new SparkFlex(RobotConstants.PORTS.CAN.INTAKE_MOTOR, MotorType.kBrushless);

            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);
            config.smartCurrentLimit(80);

            intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(IntakeInput input) {
        RobotIO.processInput(input);
        if (ENABLED) {
            if (setSpeed != input.getSpeed()) {
                intakeMotor.set(input.getSpeed());
                setSpeed = input.getSpeed();
            }
        }
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected IntakeOutput toOutputs() {
        IntakeOutput output = new IntakeOutput();

        if (ENABLED) {
            output.setIntakeMotorOutput(SparkOutput.createOutput(intakeMotor));
        }

        return output;
    }

}
