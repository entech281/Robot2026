package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;

public class TransferSubsystem extends EntechSubsystem<TransferInput, TransferOutput> {
    private static final boolean ENABLED = true;
    private static final boolean BRAKING = false;

    private double setSpeed = 0.0;

    private SparkMax transferMotor;

    @Override
    public void initialize() {
        if (ENABLED) {
            transferMotor = new SparkMax(RobotConstants.PORTS.CAN.TRANSFER_MOTOR, MotorType.kBrushless);

            SparkMaxConfig config = new SparkMaxConfig();

            config.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);

            config.secondaryCurrentLimit(30);

            transferMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void acceptInputs(TransferInput input) {
        input.log();
        if (ENABLED) {
            if (input.getSpeed() != setSpeed) {
                setSpeed = input.getSpeed();
                transferMotor.set(-input.getSpeed());
            }
        }
    }

    @Override
    public Command getTestCommand() {
        return new TestTransferCommand(this);
    }

    @Override
    protected TransferOutput toOutputs() {
        TransferOutput output = new TransferOutput();

        output.setBraking(BRAKING);
        if (ENABLED) {
            output.setTransferMotorOutput(SparkOutput.createOutput(transferMotor));
        }

        return output;
    }

}
