package frc.robot.commandgroups;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.commands.ViserSpeakerDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;


public class ShootGroup extends SequentialCommandGroup {
    public ShootGroup(Swerve swerve, Arm arm, Shooter shooter, Feeder feeder, Intake intake, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        addCommands(
            Commands.race(
                Commands.sequence(
                        Commands.parallel(
                            new SetArmAngle(arm, swerve),
                            // new ViserSpeaker(swerve).withTimeout(1),
                            Commands.race(new SetShooterSpeed(swerve, shooter), Commands.run(() -> intake.setSpeed(0.3), intake).finallyDo(() -> intake.setSpeed(0)))
                            ),
                        // Commands.race(new SetArmAngle(arm, swerve), Commands.run(() -> intake.setSpeed(0.3), intake).finallyDo(() -> intake.setSpeed(0))),
                            Commands.run(() -> feeder.setSpeed(1), feeder).withTimeout(1)
                            .finallyDo(() -> {
                                shooter.setSpeed(0);
                                intake.setSpeed(0);
                                feeder.setSpeed(0);
                            }),
                            Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.38)))
                ),
                new ViserSpeakerDrive(swerve, translationSup, strafeSup, rotationSup)
            )
            // Commands.runOnce(() -> shooter.hasNote = false)
            );
    }
}
