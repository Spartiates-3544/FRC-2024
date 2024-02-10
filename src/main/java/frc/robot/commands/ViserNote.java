package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class ViserNote extends Command{

    private PIDController align;
    private Swerve swerve;
    private ChassisSpeeds output;

    public ViserNote(Swerve swerve) {
        align = new PIDController(Constants.Swerve.alignkP, 0, 0);
        align.setSetpoint(0);
        align.setTolerance(2);
        output = new ChassisSpeeds();
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("limelight")) {
            output.omegaRadiansPerSecond = align.calculate(LimelightHelpers.getTX("limelight"));
            swerve.setChassisSpeeds(output);
        }

        SmartDashboard.putData(align);

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
