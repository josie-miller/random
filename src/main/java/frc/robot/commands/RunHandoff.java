package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handoff.Handoff;

public class RunHandoff extends Command {
    private final Handoff handoff;
    private final double voltage;
    private final double time;
    private double stateStartTime = 0;

    public RunHandoff(Handoff handoff, double voltage, double time) {
        this.handoff = handoff;
        this.voltage = voltage;
        this.time = time;
        addRequirements(handoff);
    }

    @Override
    public void initialize() {
        stateStartTime = RobotController.getFPGATime() / 1.0E6; 
        handoff.runHandoff(voltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        handoff.runHandoff(0.0);
    }

    @Override
    public boolean isFinished() {
        return (RobotController.getFPGATime() / 1.0E6 - stateStartTime > time); 
    }
}
