package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Pickup2 extends Command {
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    private Swerve swerve;
    private int counter;
    private ArrayList<Double> rpmHistory = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
    private LinearFilter rpmFilter;
    private LinearFilter sensorFilter;
    private boolean finished = false;
    private boolean sensorDetected = false;
    private double currentRpm = 0;
    private double filteredCurrent = 0;
    private double filteredProximity = 0;
    private double deltaRpm = 0;

    // private int reverseCounter = 0;

    private double speed;

    public Pickup2(Intake intake, Feeder feeder, Shooter shooter, Swerve swerve, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.speed = speed;
        this.swerve = swerve;
        rpmFilter = LinearFilter.highPass(0.1, 0.02);
        sensorFilter = LinearFilter.highPass(0.1, 0.02);
        addRequirements(intake, feeder, shooter);

    }

    @Override
    public void initialize() {
        // swerve.setLedColor(0.93);
        swerve.setLedColor(2145);
        swerve.setLedColor(1965);
        // swerve.setLedColor(0.83); //Green-Yellow
        // setLedColor(0.87); green
    }
    @Override
    public void execute() {
        currentRpm = feeder.getVelocity();

        filteredCurrent = rpmFilter.calculate(feeder.getOutputCurrent());
        filteredProximity = sensorFilter.calculate(intake.getSensorProximity());
        SmartDashboard.putNumber("filtered proximity", filteredProximity);


        rpmHistory.add((Double) currentRpm);
        rpmHistory.remove(0);
        deltaRpm = rpmHistory.get(2) - rpmHistory.get(0);
        
        if (intake.isSensorConnected()) {
            if (filteredProximity >= 80) {
                sensorDetected = true;
            } else {
                intake.setVoltage(speed * RobotController.getBatteryVoltage());
                feeder.setVoltage(speed * RobotController.getBatteryVoltage());
                shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());
            }
        
            if (sensorDetected) {
                // swerve.setLedColor(0.87);
                swerve.setLedColor(2145);
                swerve.setLedColor(1935);
                counter++;
            }

            if (counter >= 8) {
                finished = true;
            }
        } else {
            if (filteredCurrent >= 0.8 && deltaRpm <= -40) {
                finished = true;
            } else {
                intake.setVoltage(speed * RobotController.getBatteryVoltage());
                feeder.setVoltage(speed * RobotController.getBatteryVoltage());
                shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());
            }
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
        rpmHistory = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
        filteredCurrent = 0;
        currentRpm = 0;
        deltaRpm = 0;
        counter = 0;
        sensorDetected = false;
        rpmFilter.reset();
        sensorFilter.reset();
        finished = false;

        // shooter.hasNote = true;
    }


}
