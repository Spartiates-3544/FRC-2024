package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandgroups.AmpGroup;
import frc.robot.commandgroups.PassGroup;
import frc.robot.commandgroups.ShootGroup;
import frc.robot.commandgroups.ShootGroup_Auto;
import frc.robot.commandgroups.ShootGroup_auto_noaim;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Feeder feeder = new Feeder();
    private final Climber climber = new Climber();

    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final Joystick coDriver = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotation = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton amp = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton raiseClimber = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton lowerClimber = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton pass = new JoystickButton(driver, XboxController.Button.kA.value);
    
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final Trigger intakeNote = new Trigger(() -> driver.getRightTriggerAxis() >= 0.3);
    private final Trigger shoot = new Trigger(() -> driver.getLeftTriggerAxis() >= 0.3);

    private final Trigger inShootRange = new Trigger(() -> s_Swerve.getDistanceToSpeaker() <= 120 && !feeder.getBeamBreak());

    private final POVButton stopAll = new POVButton(driver, 0);
    private final POVButton reverseToggle = new POVButton(driver, 180);
    private final JoystickButton moveToAmp = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton aimNote = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Copilot Buttons */
    private final JoystickButton intakeForwards = new JoystickButton(coDriver, 11);
    private final JoystickButton intakeBackwards = new JoystickButton(coDriver, 10);
    private final JoystickButton enableArmControl = new JoystickButton(coDriver, 3);
    private final JoystickButton enableShooter = new JoystickButton(coDriver, 1);
    private final JoystickButton lowerArm = new JoystickButton(coDriver, 4);
    private final JoystickButton ampArm = new JoystickButton(coDriver, 5);
    private final JoystickButton spinUpShooter = new JoystickButton(coDriver, 2);
    

    private Boolean reverseMode = false;
    private Boolean robotCentric = false;
    private SendableChooser<Command> autoChooser;

    private HttpCamera limelight;
    private PathPlannerPath OTFAmp = PathPlannerPath.fromPathFile("Worlds_OnTheFlyAmp");

    private Trigger reverseModeTrigger = new Trigger(() -> reverseMode);

    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -Math.copySign(driver.getRawAxis(translationAxis) * driver.getRawAxis(translationAxis), driver.getRawAxis(translationAxis)) * s_Swerve.getMaxOutput(), 
                () -> -Math.copySign(driver.getRawAxis(strafeAxis) * driver.getRawAxis(strafeAxis), driver.getRawAxis(strafeAxis)) * s_Swerve.getMaxOutput(), 
                () -> -Math.copySign(driver.getRawAxis(rotation) * driver.getRawAxis(rotation), driver.getRawAxis(rotation)) * 0.35, 
                () -> robotCentric
            )
        );

        // SmartDashboard.putData(CommandScheduler.getInstance());
        // Configure the button bindings
        configureButtonBindings();
        registerCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Match").add(autoChooser).withPosition(0, 0);

        limelight = new HttpCamera("limelight", "http://10.35.44.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
        Shuffleboard.getTab("Match").add("LL", limelight).withPosition(1, 0).withSize(6, 4).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance()).withPosition(0, 0).withSize(3, 2);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Bindings */
        zeroGyro.onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));

        reverseToggle.onTrue(Commands.runOnce(() -> reverseMode = !reverseMode));
        reverseModeTrigger.onTrue(Commands.runOnce(() -> {s_Swerve.setLedColor(2145); s_Swerve.setLedColor(1885);}));
        reverseModeTrigger.onFalse(Commands.runOnce(() -> {s_Swerve.setLedColor(2145); s_Swerve.setLedColor(1965);}));

        //Intake
        intakeNote.and(() -> !reverseMode).onTrue(Commands.sequence(Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.37))).withTimeout(0.25), Commands.runOnce(() -> s_Swerve.setMaxOutput(1)), new PickupBeamBreak(intake, feeder, shooter, s_Swerve, 1).finallyDo(() -> {s_Swerve.setMaxOutput(1);}), Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, 0.5)).finallyDo(() -> driver.setRumble(RumbleType.kBothRumble, 0)).withTimeout(0.1)));
        //Outtake
        intakeNote.and(() -> reverseMode).onTrue(Commands.parallel(Commands.run(() -> intake.setSpeed(-0.3)), Commands.run(() -> feeder.setSpeed(-0.5))).withTimeout(1.5).finallyDo(() -> {intake.setSpeed(0); feeder.setSpeed(0);}));
        
        raiseClimber.onTrue(Commands.runOnce(() -> {climber.setSpeed(-0.1); arm.resetMotionMagicConstants(); arm.setAngle(Rotation2d.fromRotations(0.55));}, climber, arm));
        lowerClimber.onTrue(Commands.runOnce(() -> {climber.setSpeed(0); arm.setMotionMagicConstants(Constants.ArmConstants.MotionMagicCruiseVelocity, 1, 0.5); arm.setAngle(Rotation2d.fromRotations(0.37));}, climber, arm));

        shoot.onTrue(new ShootGroup(s_Swerve, arm, shooter, feeder, intake, () -> -driver.getRawAxis(translationAxis) * s_Swerve.getMaxOutput(), () -> -driver.getRawAxis(strafeAxis) * s_Swerve.getMaxOutput(), () -> -driver.getRawAxis(rotation) * 0.3));
        amp.onTrue(new AmpGroup(intake, arm, feeder, shooter));
        pass.onTrue(new PassGroup(s_Swerve, arm, shooter, feeder, intake));
        // moveToAmp.onTrue(AutoBuilder.pathfindToPose(new Pose2d(1.775, 7.506, Rotation2d.fromDegrees(-90)), Constants.AutoConstants.constraints, 0).withTimeout(10));
        moveToAmp.onTrue(AutoBuilder.pathfindThenFollowPath(OTFAmp, Constants.AutoConstants.constraints));

        stopAll.onTrue(Commands.parallel(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()), Commands.runOnce(() -> shooter.setSpeed(0), shooter), Commands.runOnce(() -> arm.resetMotionMagicConstants())));

        /* Codriver Bindings */
        intakeForwards.whileTrue(Commands.run(() -> {intake.setSpeed(0.5); feeder.setSpeed(0.5);}, intake, feeder).finallyDo(() -> {intake.setSpeed(0); feeder.setSpeed(0);}));
        intakeBackwards.whileTrue(Commands.run(() -> {intake.setSpeed(-0.5); feeder.setSpeed(-0.5);}, intake, feeder).finallyDo(() -> {intake.setSpeed(0); feeder.setSpeed(0);}));
        enableArmControl.toggleOnTrue(Commands.run(() -> arm.pourcentageControl(coDriver.getRawAxis(1) * 0.3), arm).finallyDo(() -> arm.setAngle(Rotation2d.fromRotations(0.39))));
        enableShooter.whileTrue(Commands.run(() -> shooter.setVelocity((coDriver.getRawAxis(2) + 1) / 2 * Constants.ShooterConstants.shooterMaxRPM), shooter).finallyDo(() -> shooter.setSpeed(0)));

        lowerArm.onTrue(Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.38)), arm));
        ampArm.onTrue(Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.63)), arm));

        spinUpShooter.toggleOnTrue(Commands.run(() -> shooter.setVelocity(3500), shooter).finallyDo(() -> shooter.setSpeed(0)));

        /* SysID Bindings */
        // intakeNote.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward).alongWith(Commands.run(() -> {
        //     s_Swerve.mSwerveMods[0].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[1].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[2].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[3].setAngleAngle(Rotation2d.fromDegrees(0));})
        //     ));
        // shoot.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse).alongWith(Commands.run(() -> {
        //     s_Swerve.mSwerveMods[0].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[1].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[2].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[3].setAngleAngle(Rotation2d.fromDegrees(0));})
        //     ));
        // amp.whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward).alongWith(Commands.run(() -> {
        //     s_Swerve.mSwerveMods[0].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[1].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[2].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[3].setAngleAngle(Rotation2d.fromDegrees(0));})
        //     ));
        // zeroGyro.whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).alongWith(Commands.run(() -> {
        //     s_Swerve.mSwerveMods[0].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[1].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[2].setAngleAngle(Rotation2d.fromDegrees(0));
        //     s_Swerve.mSwerveMods[3].setAngleAngle(Rotation2d.fromDegrees(0));})
        //     ));

        /* Range bindings */
        inShootRange.onTrue(Commands.runOnce(() -> {s_Swerve.setLedColor(2145); s_Swerve.setLedColor(1805);}));
        inShootRange.onFalse(Commands.either(Commands.runOnce(() -> {s_Swerve.setLedColor(2145); s_Swerve.setLedColor(1965);}), Commands.runOnce(() -> {s_Swerve.setLedColor(2145); s_Swerve.setLedColor(1935);}), () -> feeder.getBeamBreak()));
    }

    private void registerCommands() {
        // NamedCommands.registerCommand("placerAmp", Commands.parallel(
        //     Commands.run(() -> intake.setSpeed(0.3), intake).withTimeout(1).finallyDo(() -> intake.setSpeed(0)),
        //     Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.63)), arm),
        //     Commands.sequence(
        //         Commands.waitSeconds(2),  
        //         Commands.parallel(
        //             Commands.run(() -> feeder.setSpeed(1), feeder),
        //             Commands.run(() -> shooter.setSpeed(0.4), shooter)))
        //     ).withTimeout(3).finallyDo(() -> {
        //         shooter.setSpeed(0);
        //         intake.setSpeed(0);
        //         feeder.setSpeed(0);
        //     }));
        NamedCommands.registerCommand("placerAmp", new AmpGroup(intake, arm, feeder, shooter));
        NamedCommands.registerCommand("intake", new PickupBeamBreak(intake, feeder, shooter, s_Swerve, 1));
        NamedCommands.registerCommand("shoot", new ShootGroup_Auto(s_Swerve, arm, shooter, feeder, intake));
        NamedCommands.registerCommand("shoot_noaim", new ShootGroup_auto_noaim(s_Swerve, arm, shooter, feeder));
        NamedCommands.registerCommand("spinUpShooter", Commands.runOnce(() -> shooter.setVelocity(4450), shooter));
        NamedCommands.registerCommand("moveArmToIntake", Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.37)), arm).withTimeout(0.25));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
