package frc.robot.commands;

import frc.robot.Constants.swerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {    
    private final Swerve swerveSubsystem;    
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;

    public TeleopSwerve(Swerve swerveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), 0.05), 3); 
        double strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.05), 3); 
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.05), 1);

        double x_speed = translationVal * swerveConstants.maxSpeed;
        double y_speed = strafeVal * swerveConstants.maxSpeed;
        double rot_speed = rotationVal * swerveConstants.maxAngularVelocity;

        swerveSubsystem.requestDesiredState(
            x_speed, 
            y_speed,
            rot_speed, 
            true,
            true
        );
    }
}
