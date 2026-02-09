package frc.robot.subsystems.intake;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class IntakeOutput extends SubsystemOutput {
    private SparkOutput intakeMotorOutput;

    @Override
    protected void toLog() {
        if (intakeMotorOutput != null) {
            intakeMotorOutput.log("IntakeOutput/intakeMotor");
        }
    }

    /**
     * @return SparkOutput return the intakeMotorOutput
     */
    public SparkOutput getIntakeMotorOutput() {
        return intakeMotorOutput;
    }

    /**
     * @param intakeMotorOutput the intakeMotorOutput to set
     */
    public void setIntakeMotorOutput(SparkOutput intakeMotorOutput) {
        this.intakeMotorOutput = intakeMotorOutput;
    }

}