package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private CANSparkMax climber;
    private SparkPIDController pid;
    private RelativeEncoder encoder;

    public Climber() {
        climber = new CANSparkMax(7, MotorType.kBrushless);
        climber.restoreFactoryDefaults();
        climber.setIdleMode(IdleMode.kCoast);
        climber.setSmartCurrentLimit(30);

        encoder = climber.getEncoder();

        pid = climber.getPIDController();
        pid.setP(Constants.ClimberConstants.kP);
        pid.setI(Constants.ClimberConstants.kI);
        pid.setD(Constants.ClimberConstants.kD);
    }

    public void setIdleMode(IdleMode idleMode) {
        climber.setIdleMode(idleMode);
    }

    public void setPosition(double position) {
        pid.setReference(position, ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        climber.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber encoder", encoder.getPosition());
        // climber.set(0.5);       
    }
}
