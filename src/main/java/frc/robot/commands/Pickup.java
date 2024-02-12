package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class Pickup extends Command {
    private Intake intake;
    private Feeder feeder;
    private int counter;
    private double speed;

    public Pickup(Intake intake, Feeder feeder, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.speed = speed;
    }

    @Override
    public void execute() {
        if (feeder.getOutputCurrent() >= 13) {
            counter++;
        }
        intake.setSpeed(speed);
        feeder.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return counter >= 5;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
        feeder.setSpeed(0);
        counter = 0;
    }


}
