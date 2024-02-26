package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class ViserSpeaker extends Command{

    private PIDController align;
    private Swerve swerve;
    private ChassisSpeeds output;

    public ViserSpeaker(Swerve swerve) {
        align = new PIDController(Constants.Swerve.alignkP, Constants.Swerve.alignkI, 0);
        align.setSetpoint(0);
        align.setTolerance(1);
        output = new ChassisSpeeds();
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getFiducialID(Constants.ArmConstants.armLimelightName) == 4 || LimelightHelpers.getFiducialID(Constants.ArmConstants.armLimelightName) == 7) {
            output.omegaRadiansPerSecond = align.calculate(LimelightHelpers.getTX(Constants.ArmConstants.armLimelightName));
            swerve.setChassisSpeeds(output);
        }

        // SmartDashboard.putData(align);

    }

    @Override
    public boolean isFinished(){
        return align.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
    }

}
