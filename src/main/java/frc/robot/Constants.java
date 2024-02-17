package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(26.0);
        public static final double wheelBase = Units.inchesToMeters(26.0);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.07;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.15585;
        public static final double driveKV = 2.2565;
        public static final double driveKA = 0.24337;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.525;
        /** Radians per Second */
        public static final double maxAngularVelocity = 23.364;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double alignkP = 0.05;

        public static final String drivetrainCameraName = "limelight-ai";

        public static final double alignkI = 0.0;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // public static final class Mod0 { 
        //     public static final int driveMotorID = 11;
        //     public static final int angleMotorID = 12;
        //     public static final int canCoderID = 1;
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-39.462);
        //     public static final SwerveModuleConstants constants = 
        //         new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        // }

        // /* Front Right Module - Module 1 */
        // public static final class Mod1 { 
        //     public static final int driveMotorID = 51;
        //     public static final int angleMotorID = 52;
        //     public static final int canCoderID = 5;
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(118.125);
        //     public static final SwerveModuleConstants constants = 
        //         new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        // }

        public static final class Mod0 { 
            public static final int driveMotorID = 51;
            public static final int angleMotorID = 52;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(118.125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-39.462);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-107.05);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.08);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 4;

        public static final PathConstraints constraints = new PathConstraints(3, 3.00, Units.degreesToRadians(540.00), Units.degreesToRadians(720.00));
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final HolonomicPathFollowerConfig holonomicPathFollower = new HolonomicPathFollowerConfig(new PIDConstants(kPXController, 0, 0),
         new PIDConstants(kPThetaController, 0, 0),
         kMaxSpeedMetersPerSecond,
         Units.inchesToMeters(15),
         new ReplanningConfig());
    }

    public static final class ArmConstants {
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(124.27);
        public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.Clockwise_Positive;
        public static final double kP = 10;
        public static final double MotionMagicAcceleration = 5;
        public static final double MotionMagicCruiseVelocity = 5;
        public static final double MotionMagicJerk = 0;
        public static final String armLimelightName = "limelight";
    }

    public static final class ShooterConstants {
        public static final double kP = 0.8;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.12255;
        public static final double kV = 0.10991;
        public static final double kA = 0.0022191;

        public static final int shootCurrentLimit = 35;
        public static final int shootCurrentThreshold = 60;
        public static final double shootCurrentThresholdTime = 0.1;
        public static final boolean shootEnableCurrentLimit = true;
    }
}
