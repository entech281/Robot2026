package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SparkMaxOutput;
import frc.entech.subsystems.SubsystemOutput;

public class ShooterOutput extends SubsystemOutput {
    private double speed = 0.0;
    private boolean braking = false;
    private boolean atSpeed = true;
    private SparkMaxOutput shooterMotorA;
    private SparkMaxOutput shooterMotorB;

    @Override
    public void toLog() {
        Logger.recordOutput("ShooterOutput/speed", speed);
        Logger.recordOutput("ShooterOutput/braking", braking);
        Logger.recordOutput("ShooterOutput/atSpeed", atSpeed);
        shooterMotorA.log("ShooterOutput/motorA");
        shooterMotorB.log("ShooterOutput/motorB");
    }

    /**
     * @return double return the speed
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * @param speed the speed to set
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * @return boolean return the braking
     */
    public boolean isBraking() {
        return braking;
    }

    /**
     * @param braking the braking to set
     */
    public void setBraking(boolean braking) {
        this.braking = braking;
    }

    /**
     * @return boolean return the atSpeed
     */
    public boolean isAtSpeed() {
        return atSpeed;
    }

    /**
     * @param atSpeed the atSpeed to set
     */
    public void setAtSpeed(boolean atSpeed) {
        this.atSpeed = atSpeed;
    }

    /**
     * @return SparkMaxOutput return the shooterMotorA
     */
    public SparkMaxOutput getShooterMotorA() {
        return shooterMotorA;
    }

    /**
     * @param shooterMotorA the shooterMotorA to set
     */
    public void setShooterMotorA(SparkMaxOutput shooterMotorA) {
        this.shooterMotorA = shooterMotorA;
    }

    /**
     * @return SparkMaxOutput return the shooterMotorB
     */
    public SparkMaxOutput getShooterMotorB() {
        return shooterMotorB;
    }

    /**
     * @param shooterMotorB the shooterMotorB to set
     */
    public void setShooterMotorB(SparkMaxOutput shooterMotorB) {
        this.shooterMotorB = shooterMotorB;
    }

}
