package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class PickupBeamBreak_test extends Command {
    private Intake intake;
    private Feeder feeder;
    private Shooter shooter;
    private Swerve swerve;

    // private int reverseCounter = 0;

    private double speed;
    private boolean noteIntaked = false;
    private boolean finished = false;
    private double counter = 0;

    public PickupBeamBreak_test(Intake intake, Feeder feeder, Shooter shooter, Swerve swerve, double speed) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.speed = speed;
        this.swerve = swerve;
        addRequirements(intake, feeder, shooter);

    }

    @Override
    public void initialize() {
        swerve.setLedColor(2145);
        swerve.setLedColor(1965);
    }
    @Override
    public void execute() {
        intake.setVoltage(speed * RobotController.getBatteryVoltage());
        feeder.setVoltage(speed * RobotController.getBatteryVoltage());
        shooter.setVoltage(-0.3 * RobotController.getBatteryVoltage());

        if (!feeder.getBeamBreak()) {
            noteIntaked = true;
        }

        if (noteIntaked && counter < 15) {
            feeder.setSpeed(-speed);
            intake.setSpeed(-speed);
            counter++;
        }
        
        if (counter >= 15 && feeder.getBeamBreak()) {
            feeder.setSpeed(speed);
            intake.setSpeed(speed);
        }

        if (counter >= 15 && !feeder.getBeamBreak()) {
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
        counter = 0;
        finished = false;
        noteIntaked = false;
    }


}
