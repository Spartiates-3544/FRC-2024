package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
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
        // double angleRot = (0.0033 * Math.pow(d, 3)) - (0.025 * Math.pow(d, 2)) + (0.0717 * d) + 0.37;
        double angleRot = (0.0497 * Math.log(d)) + 0.2404;
        if (angleRot >= 0.33 && angleRot <= 0.5) {
            arm.setAngle(Rotation2d.fromRotations(angleRot));
        } else {
            arm.setAngle(Rotation2d.fromRotations(0.42));
        }
        SmartDashboard.putNumber("Arm commanded angle", angleRot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
