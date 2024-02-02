package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    // private final apriltag lime = new apriltag();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rightTrigger = XboxController.Axis.kRightTrigger.value;
    private final int leftTrigger = XboxController.Axis.kLeftTrigger.value;
    private final int rotation = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton zeroArm = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton moveArm = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton toggleIntake = new POVButton(driver, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis) * 0.3, 
                () -> -driver.getRawAxis(strafeAxis) * 0.35, 
                () -> -driver.getRawAxis(rotation) * 0.35, 
                () -> robotCentric.getAsBoolean()
            )
        );

        //arm.setDefaultCommand(Commands.run(() -> arm.pourcentageControl(() -> ((driver.getRawAxis(3) - driver.getRawAxis(2)) * 0.5)), arm));
        // Configure the button bindings
        configureButtonBindings();
        registerCommands();
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
        zeroArm.onTrue(Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.35)), arm));
        moveArm.onTrue(Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.6)), arm));

        toggleIntake.toggleOnTrue(intake.runIntake(0.5));
    }

    private void registerCommands() {
        NamedCommands.registerCommand("leverBras", Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.6))));
        NamedCommands.registerCommand("zeroBras", Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.35))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return new PathPlannerAuto("test2");
    }
}
