package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Pickup2 extends Command {
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    // private int counter;
    private double pastRpm = 0;
    private LinearFilter filter;
    private boolean finished = false;

    // private int reverseCounter = 0;

    private double speed;

    public Pickup2(Intake intake, Feeder feeder, Shooter shooter, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.speed = speed;
        filter = LinearFilter.highPass(0.1, 0.02);
    }

    @Override
    public void execute() {
        double currentRpm = feeder.getVelocity();
        double deltaRpm = currentRpm - pastRpm;
        double filteredCurrent = filter.calculate(feeder.getOutputCurrent());

        // feeder.getOutputCurrent() > 11.0 && deltaRpm <= -100
        if (filteredCurrent >= 1.0 && deltaRpm <= -100) {
            finished = true;
        } else {
            intake.setVoltage(speed * RobotController.getBatteryVoltage());
            feeder.setVoltage(speed * RobotController.getBatteryVoltage());
            shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());
        }
        pastRpm = currentRpm;
        SmartDashboard.putNumber("Filtered current", filteredCurrent);
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
        filter.reset();
        finished = false;
    }


}
