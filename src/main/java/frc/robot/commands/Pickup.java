package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class Pickup extends Command {
    private Intake intake;
    private Feeder feeder;
    private int counter;
    private LinearFilter filter;
    private boolean finished = false;

    // private int reverseCounter = 0;

    private double speed;

    public Pickup(Intake intake, Feeder feeder, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.speed = speed;
        filter = LinearFilter.singlePoleIIR(0.1, 0.02);
        addRequirements(intake, feeder);
    }

    @Override
    public void execute() {
        double outputCurrent = filter.calculate(feeder.getOutputCurrent());
        // if (outputCurrent >= 13 && counter <= 3) {
        //     intake.setSpeed(speed);
        //     feeder.setSpeed(speed);
        //     counter++;
        // } else if (reverseCounter <= 10) {
        //     intake.setSpeed(-speed);
        //     feeder.setSpeed(-speed);
        //     reverseCounter++;
        // } else {
        //     finished = true;
        // }

        if (outputCurrent >= 20) {
            counter++;
        }

        if (counter <= 3) {
            intake.setSpeed(0.4);
            feeder.setSpeed(0.3);
        // } else if (reverseCounter <= 18) {
        //     intake.setSpeed(-speed);
        //     feeder.setSpeed(-speed);
        //     reverseCounter++;
        } else {
            intake.setSpeed(0);
            feeder.setSpeed(0);
            finished = true;
        }

        // SmartDashboard.putNumber("Feeder filter current", outputCurrent);
        // SmartDashboard.putNumber("Feeder current", feeder.getOutputCurrent());
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
        feeder.setSpeed(0);
        counter = 0;
        // reverseCounter = 0;
        finished = false;
        filter.reset();
    }


}
