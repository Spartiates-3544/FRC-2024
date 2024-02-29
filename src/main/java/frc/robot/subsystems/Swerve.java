package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
// import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.sensors.PigeonIMUConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    //public PigeonIMU gyro;
    public AHRS gyro;
    public Field2d field;
    private double maxOutput = 0.6;
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public Spark blinkin;

    private SysIdRoutine characterizationRoutine;

    private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    private final MutableMeasure<Distance> m_distance = MutableMeasure.mutable(Units.Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));

    public Swerve() {
        //gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        //gyro.configAllSettings(new PigeonIMUConfiguration());
        gyro = new AHRS(SPI.Port.kMXP);
        //gyro.setYaw(0);
        gyro.reset();

        field = new Field2d();
        SmartDashboard.putData(field);

        blinkin = new Spark(0);
        setLedColor(0.93);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());

        AutoBuilder.configureHolonomic(
         this::getPose,
         this::setPose,
         this::getRobotRelativeSpeeds,
         this::setChassisSpeeds,
         Constants.AutoConstants.holonomicPathFollower,
         () -> {
            var alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }

            return false;
         },
         this);

         PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("traj").setPoses(poses));

        characterizationRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {setVoltage(volts.in(Units.Volts));},
         (SysIdRoutineLog log) -> {
            log.motor("front-left")
            .voltage(m_appliedVoltage.mut_replace(mSwerveMods[0].getDriveVoltage(), Units.Volts))
            .linearPosition(m_distance.mut_replace(mSwerveMods[0].getPosition().distanceMeters, Units.Meters))
            .linearVelocity(m_velocity.mut_replace(mSwerveMods[0].getState().speedMetersPerSecond, Units.MetersPerSecond));
            log.motor("front-right")
            .voltage(m_appliedVoltage.mut_replace(mSwerveMods[1].getDriveVoltage(), Units.Volts))
            .linearPosition(m_distance.mut_replace(mSwerveMods[1].getPosition().distanceMeters, Units.Meters))
            .linearVelocity(m_velocity.mut_replace(mSwerveMods[1].getState().speedMetersPerSecond, Units.MetersPerSecond));
            log.motor("back-left")
            .voltage(m_appliedVoltage.mut_replace(mSwerveMods[2].getDriveVoltage(), Units.Volts))
            .linearPosition(m_distance.mut_replace(mSwerveMods[2].getPosition().distanceMeters, Units.Meters))
            .linearVelocity(m_velocity.mut_replace(mSwerveMods[2].getState().speedMetersPerSecond, Units.MetersPerSecond));
            log.motor("back-right")
            .voltage(m_appliedVoltage.mut_replace(mSwerveMods[3].getDriveVoltage(), Units.Volts))
            .linearPosition(m_distance.mut_replace(mSwerveMods[3].getPosition().distanceMeters, Units.Meters))
            .linearVelocity(m_velocity.mut_replace(mSwerveMods[3].getState().speedMetersPerSecond, Units.MetersPerSecond));
         },
          this));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // SwerveModuleState[] swerveModuleStates =
        //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        //         fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //                             translation.getX(), 
        //                             translation.getY(), 
        //                             rotation, 
        //                             getHeading()
        //                         )
        //                         : new ChassisSpeeds(
        //                             translation.getX(), 
        //                             translation.getY(), 
        //                             rotation)
        //                         );
        SwerveModuleState[] swerveModuleStates;

        if (fieldRelative && DriverStation.getAlliance().get() == Alliance.Red) {
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(getHeading().getDegrees() + 180)));
        } else if (fieldRelative && DriverStation.getAlliance().get() == Alliance.Blue) {
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getHeading()));
        } else {
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(states);
    }

    public void setVoltage(double volts) {
        for(SwerveModule swerveModule : mSwerveMods) {
            swerveModule.setDriveVoltage(volts);
        }
    }

    public void setLedColor(double colorCode) {
        blinkin.set(colorCode);
    }

    public double getDistanceToSpeaker() {
        if (LimelightHelpers.getFiducialID(Constants.ArmConstants.armLimelightName) == 4 || LimelightHelpers.getFiducialID(Constants.ArmConstants.armLimelightName) == 7) {
           return (Constants.FieldConstants.speakerApriltagHeight - Constants.ArmConstants.armLimelightHeight) / Math.tan(Math.toRadians(Constants.ArmConstants.armLimelightAngle + LimelightHelpers.getTY(Constants.ArmConstants.armLimelightName)));
        }
        else return 0;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        swervePoseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public void zeroHeading(){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return characterizationRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return characterizationRoutine.dynamic(direction);
    }

    @Override
    public void periodic(){
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Gyro", -gyro.getYaw());
        SmartDashboard.putNumber("Distance to speaker", getDistanceToSpeaker());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        if (LimelightHelpers.getTV(Constants.ArmConstants.armLimelightName) && !DriverStation.isAutonomous()) {
            addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(Constants.ArmConstants.armLimelightName), (Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Capture(Constants.ArmConstants.armLimelightName) / 1000) - (LimelightHelpers.getLatency_Pipeline(Constants.ArmConstants.armLimelightName)) / 1000));
        }
    }
}