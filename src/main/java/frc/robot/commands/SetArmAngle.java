package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class SetArmAngle extends Command{
    private Arm arm;
    private Swerve swerve;

    public SetArmAngle(Arm arm, Swerve swerve) {
        this.arm = arm;
        this.swerve = swerve;
        addRequirements(arm);
    }
    
    @Override
    public void execute() {
        double d = swerve.getDistanceToSpeaker();
        // double angleRot = (0.0497 * Math.log(d)) + 0.2404;
        double angleDeg = (8E-06 * Math.pow(d, 3)) - (0.0042 * Math.pow(d, 2)) + (0.7094 * d) + 6.2373;
        double angleRot = (angleDeg + 124.8046) / 360;

        if (angleRot <= 0.6 && angleRot >= 0.4) {
            arm.setAngle(Rotation2d.fromRotations(angleRot));
            swerve.setLedColor(0.57);
            // arm.setAngle(Rotation2d.fromDegrees(angleDeg));
        } else {
            arm.setAngle(Rotation2d.fromRotations(0.42));
            swerve.setLedColor(0.77);
        }
        // SmartDashboard.putNumber("Arm commanded angle", angleRot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
