package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class SetArmAngle extends Command{
    private Arm arm;
    private Swerve swerve;
    private double angleRot;

    public SetArmAngle(Arm arm, Swerve swerve) {
        this.arm = arm;
        this.swerve = swerve;
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        double d = swerve.getDistanceToSpeaker();
        // if (d <= 80) {
        //     angleRot = (1E-06 * Math.pow(d, 3)) - (0.0002 * Math.pow(d, 2)) + (0.0142 * d) + 0.1393;
        // } else {
        //     angleRot = ((-1E-05) * Math.pow(d, 2)) + (0.0022 * d) + 0.3550;
        // }

        //double angleRot = ((-1E-05) * Math.pow(d, 2)) + (0.0022 * d) + 0.3550;
        double angleRot = (0.0497 * Math.log(d)) + 0.2404;
        // double angleDeg = (3E-06 * Math.pow(d, 3)) - (0.0015 * Math.pow(d, 2)) + (0.2941 * d) + (176.6186 - 34.7976);
        if (angleRot <= 0.6 && angleRot >= 0.4) {
            arm.setAngle(Rotation2d.fromRotations(angleRot));
            // arm.setAngle(Rotation2d.fromDegrees(angleDeg));
        } else {
            arm.setAngle(Rotation2d.fromRotations(0.42));
        }
        // SmartDashboard.putNumber("Arm commanded angle", angleRot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
