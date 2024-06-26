package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;


public class ShootGroup_auto_noaim extends SequentialCommandGroup {
    public ShootGroup_auto_noaim(Swerve swerve, Arm arm, Shooter shooter, Feeder feeder) {
        addCommands(
            Commands.sequence(
                    Commands.parallel(
                        new SetArmAngle(arm, swerve),
                        // new ViserSpeaker(swerve).withTimeout(1),
                        new SetShooterSpeed(swerve, shooter)
                        ),
                    // Commands.race(new SetArmAngle(arm, swerve), Commands.run(() -> intake.setSpeed(0.3), intake).finallyDo(() -> intake.setSpeed(0))),
                        Commands.run(() -> feeder.setSpeed(1), feeder).withTimeout(1)
                        .finallyDo(() -> {
                            shooter.setVelocity(4500);
                            feeder.setSpeed(0);
                        }),
                        Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.38)))
            )
            // Commands.runOnce(() -> shooter.hasNote = false)
            );
    }
}
