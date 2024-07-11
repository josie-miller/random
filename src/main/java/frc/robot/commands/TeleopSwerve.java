package frc.robot.commands;

import frc.robot.constants.swerveConstants;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.Swerve;

public class TeleopSwerve extends Command {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;


    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerve = swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), 0.05), 3); 
        double strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.05), 3) ; 
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.05), 1); 

        double x_speed = translationVal * swerveConstants.maxSpeed;
        double y_speed = strafeVal * swerveConstants.maxSpeed;
        double rot_speed = rotationVal * swerveConstants.maxAngularVelocity;

        /* Drive */
        swerve.requestDesiredState(
            x_speed, 
            y_speed,
            rot_speed, 
            true,
            true
        );
    }
}