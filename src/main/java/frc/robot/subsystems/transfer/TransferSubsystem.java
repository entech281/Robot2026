package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.livetuning.LiveTuningHandler;

public class TransferSubsystem extends EntechSubsystem<TransferInput, TransferOutput> {
    private static final boolean ENABLED = true;
    private static final boolean BRAKING = false;

    private double setSpeed = 0.0;
    private double setFlexSpeed = 0.0;


    private SparkMax transferMotor;
    private SparkFlex transferFlexMotor;

    @Override
    public void initialize() {
        if (ENABLED) {
            transferMotor = new SparkMax(RobotConstants.PORTS.CAN.TRANSFER_MOTOR, MotorType.kBrushless);
            transferFlexMotor = new SparkFlex(RobotConstants.PORTS.CAN.TRANSFER_FLEX_MOTOR, MotorType.kBrushless);

            SparkMaxConfig config = new SparkMaxConfig();
            SparkFlexConfig flexConfig = new SparkFlexConfig();


            config.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);
            flexConfig.idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast);


            config.secondaryCurrentLimit(30);
            flexConfig.secondaryCurrentLimit(30);


            transferMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            transferFlexMotor.configure(flexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(TransferInput input) {
        RobotIO.processInput(input);

        if (ENABLED) {
            updateFlexSpeed(input);

            if (input.getSpeed() != setSpeed) {
                setSpeed = input.getSpeed();
                transferMotor.set(-input.getSpeed());
            }
            if (input.getFlexSpeed() != setFlexSpeed) {
                setFlexSpeed = input.getFlexSpeed();
                transferFlexMotor.set(-setFlexSpeed);
            }
        }
    }
    private void updateFlexSpeed(TransferInput input) {
        double flexSpeed = LiveTuningHandler.getInstance().getValue(RobotConstants.TRANSFER.FLEX_SPEED_KEY);
        if (input.getSpeed() == 0.0 || !Double.isFinite(flexSpeed)) {
            input.setFlexSpeed(0.0);
        } else {
            input.setFlexSpeed(Math.signum(input.getSpeed()) * MathUtil.clamp(flexSpeed,
                    -1, 1));
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
            output.setTransferFlexMotorOutput(SparkOutput.createOutput(transferFlexMotor));

            output.setTransferMotorOutput(SparkOutput.createOutput(transferMotor));
        }

        return output;
    }

}
