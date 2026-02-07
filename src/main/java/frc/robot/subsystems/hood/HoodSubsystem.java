package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.RobotConstants;
import frc.robot.subsystems.turret.TurretInput;

public class HoodSubsystem extends EntechSubsystem<HoodInput, HoodOutput> {

    private static final boolean ENABLED = true;

    private SparkMax hoodMotor;
    private SparkClosedLoopController hoodPIDController;
    private RelativeEncoder hoodEncoder;
    private HoodInput latestInput = new HoodInput();
    private SparkMaxConfig hoodConfig;


    @Override
    public void initialize() {
            if (!ENABLED) return;
            hoodMotor = new SparkMax(RobotConstants.PORTS.CAN.HOOD_MOTOR, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
            hoodConfig = new SparkMaxConfig();
            // Make encoder report degrees directly (adjust if your encoder reports rotations)
            hoodConfig.encoder.positionConversionFactor(RobotConstants.HOOD.POSITION_CONVERSION_FACTOR_DEGREES);
    
            // Closed-loop PIDF
            hoodConfig.closedLoop
                .pid(RobotConstants.HOOD.HOOD_P, RobotConstants.HOOD.HOOD_I, RobotConstants.HOOD.HOOD_D, ClosedLoopSlot.kSlot0);
    
            // Apply conservative signals update rates similar to other subsystems
            hoodConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / 50.0));
    
            // Configure the motor with these settings
            hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    
            hoodEncoder = hoodMotor.getEncoder();

            hoodPIDController = hoodMotor.getClosedLoopController();
            hoodPIDController.setSetpoint(hoodEncoder.getPosition(), ControlType.kPosition);
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(HoodInput input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public Command getTestCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTestCommand'");
    }

    @Override
    protected HoodOutput toOutputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'toOutputs'");
    }
    
}
