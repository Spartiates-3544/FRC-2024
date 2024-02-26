package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.ViserSpeaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;


public class ShootGroup extends SequentialCommandGroup {
    public ShootGroup(Swerve swerve, Arm arm, Shooter shooter, Feeder feeder, Intake intake) {
        addCommands(
            Commands.parallel(
                new SetArmAngle(arm, swerve),
                new ViserSpeaker(swerve).withTimeout(1),
                Commands.run(() -> shooter.setVelocity(4700), shooter).withTimeout(1),
                Commands.run(() -> intake.setSpeed(0.3), intake).withTimeout(1).finallyDo(() -> intake.setSpeed(0))
            ),
            Commands.sequence(
                Commands.waitSeconds(1),  
                Commands.run(() -> feeder.setSpeed(1), feeder).withTimeout(1))
            .finallyDo(() -> {
                shooter.setSpeed(0);
                intake.setSpeed(0);
                feeder.setSpeed(0);
            }));
    }
}
