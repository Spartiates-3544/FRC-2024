package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class ViserSpeakerDrive extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private PIDController align;

    public ViserSpeakerDrive(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        align = new PIDController(Constants.Swerve.alignkP, Constants.Swerve.alignkI, Constants.Swerve.alignkD);
        align.setSetpoint(-2);
        align.setTolerance(0.25);
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
        this.strafeSup = strafeSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive with tracking*/
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            (rotationVal * Constants.Swerve.maxAngularVelocity) + align.calculate(s_Swerve.getSpeakerAngle()), 
            true, 
            true
        );
    }
}