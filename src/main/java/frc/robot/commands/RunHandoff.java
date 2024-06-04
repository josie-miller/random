package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.handoff.Handoff;

public class RunHandoff extends Command {
    private final Handoff handoff;
    private final double voltage;

    public RunHandoff(Handoff handoff, double voltage) {
        this.handoff = handoff;
        this.voltage = voltage;
        addRequirements(handoff);
    }

    @Override
    public void initialize() {
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
        return false; 
    }
}
