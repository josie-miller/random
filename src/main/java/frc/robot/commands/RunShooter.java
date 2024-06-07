package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj.RobotController;

public class RunShooter extends Command {
    private final Shooter shooter;
    private final double velocity;
    private double stateStartTime = 0;
    private double ratio;

    public RunShooter(Shooter shooter, double velocity, double ratio) {
        this.shooter = shooter;
        this.velocity = velocity;
        this.ratio = ratio;
    
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        stateStartTime = RobotController.getFPGATime() / 1.0E6; 
        shooter.runShooter(velocity, ratio);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}