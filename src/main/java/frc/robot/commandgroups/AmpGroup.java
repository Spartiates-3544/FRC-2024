package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AmpGroup extends SequentialCommandGroup{
    public AmpGroup(Intake intake, Arm arm, Feeder feeder, Shooter shooter) {
        addCommands(
            Commands.run(() -> arm.setAngle(Rotation2d.fromRotations(0.65)), arm).until(() -> arm.getAngle().getRotations() >= 0.65 - 0.01),
            Commands.run(() -> {shooter.setSpeed(0.4); feeder.setSpeed(1);}, shooter, feeder).withTimeout(1),
            Commands.runOnce(() -> {intake.setSpeed(0); feeder.setSpeed(0); shooter.setSpeed(0);}, intake, feeder, shooter)
        );


    }
}
