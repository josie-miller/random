package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class runShooter extends Command {
    private final Shooter shooter;
    private final double voltage;
    private double stateStartTime = 0;


    public runShooter(Shooter shooter, double voltage) {
        this.shooter = shooter;
        this.voltage = voltage;
        addRequirements(shooter);
      }

    @Override
    public void initialize() {
        stateStartTime = RobotController.getFPGATime() / 1.0E6; 
        shooter.setVoltage(voltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(0.0);
    }

   
}