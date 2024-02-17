package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class Pickup extends Command {
    private Intake intake;
    private Feeder feeder;
    private int counter;

    private int reverseCounter = 0;

    private double speed;

    public Pickup(Intake intake, Feeder feeder, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.speed = speed;
    }

    @Override
    public void execute() {
        if (feeder.getOutputCurrent() >= 14.5) {
            counter++;
        }
        intake.setSpeed(speed);
        feeder.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return counter >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        while (reverseCounter < 10) {
            intake.setSpeed(-speed);
            reverseCounter++;
        }

        intake.setSpeed(0);
        feeder.setSpeed(0);
        counter = 0;
    }


}
