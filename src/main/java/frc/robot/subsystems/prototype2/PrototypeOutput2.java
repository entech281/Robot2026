package frc.robot.subsystems.prototype2;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class PrototypeOutput2 extends SubsystemOutput {

    private SparkOutput so1;
    @Override
    protected void toLog() {
        so1.log("prototypeSubsystem2/motor");
    }
    public void setSparkOutput(SparkOutput so1){
        this.so1 = so1;
    }
    

}
