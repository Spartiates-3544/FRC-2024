package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase  {
    private CANSparkMax intakeMotor;
    private PicoColorSensor colorSensor;

    public Intake() {
        intakeMotor = new CANSparkMax(4, MotorType.kBrushless);
        colorSensor = new PicoColorSensor();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Intake encoder", intakeMotor.getEncoder().getPosition());
        // SmartDashboard.putNumber("Color sensor proximity", colorSensor.getProximity1());
        // SmartDashboard.putBoolean("Color sensor connected", colorSensor.isSensor1Connected());
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public double getVelocity() {
        return intakeMotor.getEncoder().getVelocity();
    }

    public double getOutputCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public double getSensorProximity() {
        return colorSensor.getProximity1();
    }

    public boolean isSensorConnected() {
        return colorSensor.isSensor1Connected();
    }

    public Command runIntake(double speed) {
        return this.run(() -> intakeMotor.set(speed)).finallyDo(() -> intakeMotor.stopMotor());
    }
}
