package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private TalonFX shooter1;
    private TalonFX shooter2;
    private TalonFXConfiguration motorConfig;
    private VelocityVoltage velocityVoltage;
    private SimpleMotorFeedforward feedforward;

    private SysIdRoutine characterizationRoutine;

    //Characterization stuff I dont understand
    private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Units.Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));

    public Shooter() {
        shooter1 = new TalonFX(4);
        shooter2 = new TalonFX(5);
        motorConfig = new TalonFXConfiguration();
        velocityVoltage = new VelocityVoltage(0);
        feedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV, Constants.ShooterConstants.kA);

        characterizationRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {shooter1.setVoltage(volts.in(Units.Volts));},
         (SysIdRoutineLog log) -> {
            log.motor("shooter-wheel")
            .voltage(m_appliedVoltage.mut_replace(shooter1.getMotorVoltage().getValueAsDouble(), Units.Volts))
            .angularPosition(m_angle.mut_replace(shooter1.getPosition().getValueAsDouble(), Units.Rotations))
            .angularVelocity(m_velocity.mut_replace(shooter1.getVelocity().getValueAsDouble(), Units.RotationsPerSecond));
         },
          this));

        configMotors();
    }

    public void configMotors() {
        //Reset to factory default
        shooter1.getConfigurator().apply(new TalonFXConfiguration());
        shooter2.getConfigurator().apply(new TalonFXConfiguration());
        //Config settings
        shooter2.setControl(new Follower(4, false));

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.ShooterConstants.shootEnableCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.shootCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = Constants.ShooterConstants.shootCurrentThreshold;
        motorConfig.CurrentLimits.SupplyTimeThreshold = Constants.ShooterConstants.shootCurrentThresholdTime;
        motorConfig.Slot0.kP = Constants.ShooterConstants.kP;
        motorConfig.Slot0.kI = Constants.ShooterConstants.kI;
        motorConfig.Slot0.kD = Constants.ShooterConstants.kD;

        shooter1.getConfigurator().apply(motorConfig);
        shooter2.getConfigurator().apply(motorConfig);
    }

    public void setSpeed(double speed) {
        shooter1.set(speed);
    }

    public void setVelocity(double rpm) {
        velocityVoltage.Velocity = (rpm / 60);
        velocityVoltage.FeedForward = feedforward.calculate(rpm / 60);
        shooter1.setControl(velocityVoltage);
    }

    public void setVoltage(double voltage) {
        shooter1.setVoltage(voltage);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return characterizationRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return characterizationRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter speed", (shooter1.getVelocity().getValueAsDouble() * 60));
    }
}
