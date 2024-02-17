package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private TalonFX bras1;
    private TalonFX bras2;
    private CANcoder encodeur;
    private CANcoderConfiguration encoderConfig;
    private TalonFXConfiguration motorConfig;

    public Arm() {
        bras1 = new TalonFX(2);
        bras2 = new TalonFX(3);
        encodeur = new CANcoder(1);
        encoderConfig = new CANcoderConfiguration();
        motorConfig = new TalonFXConfiguration();
        configMotors();
        configEncoder();
    }

    private void configMotors() {
        bras2.setControl(new Follower(2, true));
        bras1.getConfigurator().apply(motorConfig);
        bras2.getConfigurator().apply(motorConfig);
        motorConfig.Feedback.FeedbackRemoteSensorID = encodeur.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.35;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.84;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.Slot0.kP = Constants.ArmConstants.kP;
        motorConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmConstants.MotionMagicAcceleration;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmConstants.MotionMagicCruiseVelocity;
        motorConfig.MotionMagic.MotionMagicJerk = Constants.ArmConstants.MotionMagicJerk;
        bras1.getConfigurator().apply(motorConfig);
        bras2.setNeutralMode(NeutralModeValue.Coast);
        // bras2.getConfigurator().apply(motorConfig);
    }

    private void configEncoder() {
        encoderConfig.MagnetSensor.SensorDirection = Constants.ArmConstants.cancoderInvert;
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encodeur.getConfigurator().apply(encoderConfig);
    }

    public void pourcentageControl(DoubleSupplier percentage) {
        bras1.set(percentage.getAsDouble());
    }

    public void setAngle(Rotation2d angle) {
        MotionMagicDutyCycle request = new MotionMagicDutyCycle(0, false, 0, 0, false, false, false);
        bras1.setControl(request.withPosition(angle.getRotations()));
    }

    public void periodic() {
        // SmartDashboard.putNumber("Encodeur bras", (encodeur.getAbsolutePosition().getValue() - Constants.ArmConstants.angleOffset.getRotations()) * 360);
        SmartDashboard.putNumber("Arm rotation", bras1.getPosition().getValueAsDouble());
    }
}
