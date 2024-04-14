package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;


public class PassGroup extends SequentialCommandGroup {
    public PassGroup(Swerve swerve, Arm arm, Shooter shooter, Feeder feeder, Intake intake) {
        double armRot = 0.45;

        addCommands(
                Commands.sequence(
                        Commands.parallel(
                            Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(armRot)), arm).until(() -> arm.getAngle().getRotations() >= armRot - 0.01),
                            // new ViserSpeaker(swerve).withTimeout(1),
                            Commands.race(Commands.run(() -> shooter.setVelocity(5000), shooter).until(() -> shooter.atRPMSetpoint()), Commands.run(() -> intake.setSpeed(0.3), intake).finallyDo(() -> intake.setSpeed(0)))
                            ),
                        // Commands.race(new SetArmAngle(arm, swerve), Commands.run(() -> intake.setSpeed(0.3), intake).finallyDo(() -> intake.setSpeed(0))),
                            Commands.run(() -> feeder.setSpeed(1), feeder).withTimeout(1)
                            .finallyDo(() -> {
                                shooter.setSpeed(0);
                                intake.setSpeed(0);
                                feeder.setSpeed(0);
                            }),
                            Commands.runOnce(() -> arm.setAngle(Rotation2d.fromRotations(0.38)))
                )
            // Commands.runOnce(() -> shooter.hasNote = false)
            );
    }
}
