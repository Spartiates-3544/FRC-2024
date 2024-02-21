package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private CANSparkMax feeder;

    public Feeder() {
        feeder = new CANSparkMax(6, MotorType.kBrushless);
        feeder.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder encoder", feeder.getEncoder().getPosition());
    }

    public void setSpeed(double speed) {
        feeder.set(speed);
    }

    public double getOutputCurrent() {
        return feeder.getOutputCurrent();
    }

    public Command runFeeder(double speed) {
        return this.run(() -> feeder.set(speed)).finallyDo(() -> feeder.stopMotor());
    }
}
