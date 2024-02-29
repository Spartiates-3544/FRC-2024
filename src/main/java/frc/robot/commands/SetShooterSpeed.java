package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends Command {
    private Swerve swerve;
    private Shooter shooter;

    public SetShooterSpeed(Swerve swerve, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (swerve.getDistanceToSpeaker() <= 124) {
            shooter.setVelocity(4450);
        } else {
            shooter.setVelocity(4450);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
