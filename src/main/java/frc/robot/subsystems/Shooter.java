package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        //shooter2.setControl(new Follower(4, false));
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.Slot0.kP = Constants.ShooterConstants.kP;
        shooter1.getConfigurator().apply(motorConfig);
        //shooter2.getConfigurator().apply(motorConfig);
    }

    public void setSpeed(double speed) {
        shooter1.set(speed);
    }

    public void setVelocity(double rpm) {
        VelocityVoltage request = new VelocityVoltage(rpm / 60, 50, false, 0, 0, false, false, false);
        shooter1.setControl(request.withVelocity(rpm / 60));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter speed", shooter1.getVelocity().getValueAsDouble());
    }
}
