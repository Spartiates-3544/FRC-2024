package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Pickup_intakesensing extends Command {
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    private int counter = 0;
    private ArrayList<Double> rpmHistory = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
    private LinearFilter filter;
    private boolean finished = false;
    private boolean note = false;

    // private int reverseCounter = 0;

    private double speed;

    public Pickup_intakesensing(Intake intake, Feeder feeder, Shooter shooter, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.speed = speed;
        filter = LinearFilter.highPass(0.1, 0.02);
        addRequirements(intake, feeder, shooter);
    }

    @Override
    public void initialize() {
        feeder.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void execute() {
        double currentRpm = feeder.getVelocity();
        double filteredCurrent = filter.calculate(feeder.getOutputCurrent());

        rpmHistory.add((Double) currentRpm);
        rpmHistory.remove(0);
        double deltaRpm = rpmHistory.get(2) - rpmHistory.get(0);
        
        // feeder.getOutputCurrent() > 11.0 && deltaRpm <= -100
        if (filteredCurrent >= 0.4 && deltaRpm <= -40) {
            note = true;
        } else {
            intake.setVoltage(speed * RobotController.getBatteryVoltage());
            feeder.setVoltage((speed * RobotController.getBatteryVoltage()) * 0.5);
            shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());
        }
        // SmartDashboard.putNumber("Filtered current", filteredCurrent);

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
        feeder.setIdleMode(IdleMode.kBrake);
        rpmHistory = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
        filter.reset();
        finished = false;
        note = false;
        counter = 0;
    }


}
