package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX shooter1;
    private TalonFX shooter2;
    private TalonFXConfiguration motorConfig;

    public Shooter() {
        shooter1 = new TalonFX(4);
        shooter2 = new TalonFX(5);
        motorConfig = new TalonFXConfiguration();
        configMotors();
    }

    public void configMotors() {
        //Reset to factory default
        shooter1.getConfigurator().apply(new TalonFXConfiguration());
        shooter2.getConfigurator().apply(new TalonFXConfiguration());
        //Config settings
        shooter2.setControl(new Follower(4, false));
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter1.getConfigurator().apply(motorConfig);
        shooter2.getConfigurator().apply(motorConfig);
    }

    public void setSpeed(double speed) {
        shooter1.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter speed", shooter1.get());
    }
}
