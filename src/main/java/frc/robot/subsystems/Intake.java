package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase  {
    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(4, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake encoder", intakeMotor.getEncoder().getPosition());
    }

    public Command runIntake(double speed) {
        return this.run(() -> intakeMotor.set(speed)).finallyDo(() -> intakeMotor.stopMotor());
    }
}
