package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase  {
    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(4, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(20);
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

    public Command runIntake(double speed) {
        return this.run(() -> intakeMotor.set(speed)).finallyDo(() -> intakeMotor.stopMotor());
    }
}
