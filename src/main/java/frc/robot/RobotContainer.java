package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commandgroups.ShootGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Feeder feeder = new Feeder();

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
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final Trigger intakeNote = new Trigger(() -> driver.getRightTriggerAxis() >= 0.3);
    private final Trigger shoot = new Trigger(() -> driver.getLeftTriggerAxis() >= 0.3);

    /* Copilot Buttons */
    private final JoystickButton intakeForwards = new JoystickButton(coDriver, 6);
    private final JoystickButton intakeBackwards = new JoystickButton(coDriver, 7);
    private final JoystickButton enableArmControl = new JoystickButton(coDriver, 3);
    private final JoystickButton enableShooter = new JoystickButton(coDriver, 11);

    private final POVButton stopAll = new POVButton(driver, 0);
    private final POVButton reverseToggle = new POVButton(driver, 180);
    private final POVButton moveToAmp = new POVButton(driver, 90);
    private final JoystickButton aimNote = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private Boolean reverseMode = false;
    private Boolean robotCentric = false;
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis) * s_Swerve.getMaxOutput(), 
                () -> -driver.getRawAxis(strafeAxis) * s_Swerve.getMaxOutput(), 
                () -> -driver.getRawAxis(rotation) * 0.3, 
                () -> robotCentric
            )
        );

        SmartDashboard.putData(CommandScheduler.getInstance());
        // Configure the button bindings
        configureButtonBindings();
        registerCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
        reverseToggle.onTrue(Commands.runOnce(() -> reverseMode = !reverseMode));

        //Intake
        intakeNote.and(() -> !reverseMode).onTrue(Commands.sequence(Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.38))).withTimeout(0.5), Commands.runOnce(() -> s_Swerve.setMaxOutput(0.5)), new Pickup2(intake, feeder, shooter, 0.5).finallyDo(() -> {s_Swerve.setMaxOutput(0.6);})));
        //Outtake
        intakeNote.and(() -> reverseMode).onTrue(Commands.parallel(Commands.run(() -> intake.setSpeed(-0.3)), Commands.run(() -> feeder.setSpeed(-0.5))).withTimeout(1.5).finallyDo(() -> {intake.setSpeed(0); feeder.setSpeed(0);}));

        //WHAT THE HECK??? Too lazy to put this in a separate file
        // shoot.onTrue(Commands.parallel(
        //     Commands.run(() -> shooter.setVelocity(4500), shooter),
        //     new ViserSpeaker(s_Swerve).withTimeout(1),
        //     Commands.run(() -> intake.setSpeed(0.3), intake).withTimeout(1).finallyDo(() -> intake.setSpeed(0)),
        //     Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.47)), arm),
        //     Commands.sequence(
        //         Commands.waitSeconds(2),  
        //         Commands.run(() -> feeder.setSpeed(0.7), feeder)
        //         )
        //     ).withTimeout(5).finallyDo(() -> {
        //         shooter.setSpeed(0);
        //         intake.setSpeed(0);
        //         feeder.setSpeed(0);
        //     }));
        shoot.onTrue(new ShootGroup(s_Swerve, arm, shooter, feeder, intake));

        amp.onTrue(Commands.parallel(
            Commands.run(() -> intake.setSpeed(0.3), intake).withTimeout(1).finallyDo(() -> intake.setSpeed(0)),
            Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.63)), arm),
            Commands.sequence(
                Commands.waitSeconds(2),  
                Commands.parallel(
                    Commands.run(() -> feeder.setSpeed(1), feeder),
                    Commands.run(() -> shooter.setSpeed(0.4), shooter)))
            ).withTimeout(3).finallyDo(() -> {
                shooter.setSpeed(0);
                intake.setSpeed(0);
                feeder.setSpeed(0);
            }));

        aimNote.toggleOnTrue(new ViserNoteDrive(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis) * 0.75, 
                () -> -driver.getRawAxis(strafeAxis) * 0.75, 
                () -> -driver.getRawAxis(rotation) * 0.20, 
                () -> robotCentric
            ));

        stopAll.onTrue(Commands.parallel(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()), Commands.runOnce(() -> shooter.setSpeed(0), shooter)));

        moveToAmp.onTrue(AutoBuilder.pathfindToPose(new Pose2d(1.84, 7.13, Rotation2d.fromDegrees(-90)), Constants.AutoConstants.constraints, 0).withTimeout(10));

        /* Codriver Bindings */
        intakeForwards.whileTrue(Commands.run(() -> {intake.setSpeed(0.5); feeder.setSpeed(0.5);}, intake, feeder).finallyDo(() -> {intake.setSpeed(0); feeder.setSpeed(0);}));
        intakeBackwards.whileTrue(Commands.run(() -> {intake.setSpeed(-0.5); feeder.setSpeed(-0.5);}, intake, feeder).finallyDo(() -> {intake.setSpeed(0); feeder.setSpeed(0);}));
        //Arm control
        enableArmControl.toggleOnTrue(Commands.run(() -> arm.pourcentageControl(coDriver.getRawAxis(1) * 0.3), arm).finallyDo(() -> arm.setAngle(Rotation2d.fromRotations(0.39))));
        //Shooter control
        enableShooter.whileTrue(Commands.run(() -> shooter.setVelocity((coDriver.getRawAxis(2) + 1) / 2 * Constants.ShooterConstants.shooterMaxRPM), shooter).finallyDo(() -> shooter.setSpeed(0)));

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
    }

    private void registerCommands() {
        NamedCommands.registerCommand("placerAmp", Commands.parallel(
            Commands.run(() -> intake.setSpeed(0.3), intake).withTimeout(1).finallyDo(() -> intake.setSpeed(0)),
            Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.63)), arm),
            Commands.sequence(
                Commands.waitSeconds(2),  
                Commands.parallel(
                    Commands.run(() -> feeder.setSpeed(1), feeder),
                    Commands.run(() -> shooter.setSpeed(0.4), shooter)))
            ).withTimeout(3).finallyDo(() -> {
                shooter.setSpeed(0);
                intake.setSpeed(0);
                feeder.setSpeed(0);
            }));

        NamedCommands.registerCommand("intake", new Pickup2(intake, feeder, shooter, 0.6));
        NamedCommands.registerCommand("shoot", new ShootGroup(s_Swerve, arm, shooter, feeder, intake));
        NamedCommands.registerCommand("moveArmToIntake", Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.38)), arm).withTimeout(0.5));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
