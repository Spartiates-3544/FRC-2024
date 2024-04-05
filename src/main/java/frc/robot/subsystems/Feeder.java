package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private CANSparkMax feeder;
    private DigitalInput beamBreak;

    public Feeder() {
        beamBreak = new DigitalInput(0);
        feeder = new CANSparkMax(6, MotorType.kBrushless);
        feeder.setIdleMode(IdleMode.kBrake);
        feeder.setSmartCurrentLimit(15);
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Feeder encoder", feeder.getEncoder().getPosition());
        // SmartDashboard.putNumber("Feeder current", feeder.getOutputCurrent());
        // SmartDashboard.putNumber("Feeder rpm", feeder.getEncoder().getVelocity());
        SmartDashboard.putBoolean("beam break", beamBreak.get());
    }

    public boolean getBeamBreak() {
        return beamBreak.get();
    }

    public void setSpeed(double speed) {
        feeder.set(speed);
    }

    public void setVoltage(double voltage) {
        feeder.setVoltage(voltage);
    }

    public double getOutputCurrent() {
        return feeder.getOutputCurrent();
    }

    public double getVelocity() {
        return feeder.getEncoder().getVelocity();
    }

    public void setIdleMode(IdleMode mode) {
        feeder.setIdleMode(mode);
    }

    public Command runFeeder(double speed) {
        return this.run(() -> feeder.set(speed)).finallyDo(() -> feeder.stopMotor());
    }
}
