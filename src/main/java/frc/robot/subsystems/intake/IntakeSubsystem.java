package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;

public class IntakeSubsystem extends EntechSubsystem<IntakeInput, IntakeOutput> {
    private static final boolean ENABLED = true;
    private SparkFlex intakeMotor;
    private double setSpeed = 0.0;

    @Override
    public void initialize() {
        if (ENABLED) {
            intakeMotor = new SparkFlex(RobotConstants.PORTS.CAN.INTAKE_MOTOR, MotorType.kBrushless);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(IntakeInput input) {
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
