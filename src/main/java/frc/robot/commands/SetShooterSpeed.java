package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends Command {
    private Swerve swerve;
    private Shooter shooter;
    private double counter;

    public SetShooterSpeed(Swerve swerve, Shooter shooter) {
        this.swerve = swerve;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (swerve.getDistanceToSpeaker() <= 95) {
            shooter.setVelocity(4500);
        } else if(swerve.getDistanceToSpeaker() <= 200){
            shooter.setVelocity(5500);
        } else {
            shooter.setVelocity(5800);
        }

        if (shooter.atRPMSetpoint()) {
            counter++;
        }
    }

    @Override
    public boolean isFinished() {
        return counter >= 20;
    }

    @Override
    public void end(boolean interrupted) {
        counter = 0;
    }
}
