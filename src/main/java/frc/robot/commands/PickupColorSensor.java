package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PickupColorSensor extends Command {
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    private int counter;
    // private ArrayList<Double> rpmHistory = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
    // private LinearFilter filter;
    private boolean finished = false;
    private boolean note = false;
    // private double currentRpm = 0;
    // private double filteredCurrent = 0;
    // private double deltaRpm = 0;

    // private int reverseCounter = 0;

    private double speed;

    public PickupColorSensor(Intake intake, Feeder feeder, Shooter shooter, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.speed = speed;
        // filter = LinearFilter.highPass(0.1, 0.02);
        addRequirements(intake, feeder, shooter);
    }

    @Override
    public void execute() {
        // currentRpm = feeder.getVelocity();
        // filteredCurrent = filter.calculate(feeder.getOutputCurrent());

        // rpmHistory.add((Double) currentRpm);
        // rpmHistory.remove(0);
        // deltaRpm = rpmHistory.get(2) - rpmHistory.get(0);
        
        // feeder.getOutputCurrent() > 11.0 && deltaRpm <= -100
        // if (filteredCurrent >= 0.8 && deltaRpm <= -40) {
        //     finished = true;
        // } else {
        //     intake.setVoltage(speed * RobotController.getBatteryVoltage());
        //     feeder.setVoltage(speed * RobotController.getBatteryVoltage());
        //     shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());
        // }

        if (intake.getSensorProximity() >= 300) {
            note = true;
        } else {
            intake.setVoltage(speed * RobotController.getBatteryVoltage());
            feeder.setVoltage(speed * RobotController.getBatteryVoltage());
            shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());
        }

        if (note) {
            counter++;
        }

        if (counter >= 5) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
        feeder.setSpeed(0);
        shooter.setSpeed(0);
        finished = false;
        counter = 0;
        note = false;
    }


}
