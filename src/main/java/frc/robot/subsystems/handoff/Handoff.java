package frc.robot.subsystems.handoff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Handoff extends SubsystemBase {
    private final HandoffIO handoffIO;
    private double setpointVolts;

    public Handoff(HandoffIO handoffIO) {
        this.handoffIO = handoffIO;
        setpointVolts = 0.0;
    }

    public void runHandoff(double voltage) {
        setpointVolts = voltage;
        handoffIO.setOutput(voltage);
    }

    @Override
    public void periodic() {
        HandoffIO.HandoffIOInputs inputs = new HandoffIO.HandoffIOInputs();
        handoffIO.updateInputs(inputs);
        SmartDashboard.putNumber("Handoff Voltage", setpointVolts);
        SmartDashboard.putNumber("Handoff Current", inputs.current);
        SmartDashboard.putNumber("Handoff Temperature", inputs.temp);
        SmartDashboard.putNumber("Handoff Speed (RPS)", inputs.RPS);
    }
}