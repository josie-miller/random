package frc.robot.subsystems.handoff;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Handoff extends SubsystemBase {
    private final HandoffIO handoffIO;
    private HandoffIOInputsAutoLogged inputs = new HandoffIOInputsAutoLogged();
    private double setpointVolts;

    public Handoff(HandoffIO handoffIO) {
        this.handoffIO = handoffIO;
        setpointVolts = 0.0;
    }

    public void periodic(){
        handoffIO.updateInputs(inputs);
        Logger.processInputs("Handoff", inputs);
    }

    public void runHandoff(double voltage) {
        setpointVolts = voltage;
        handoffIO.runHandoff(setpointVolts);
    }

    public double getStatorCurrent(){
        return inputs.currentAmps;
    }

    public void updateInputs(HandoffIO.HandoffIOInputs inputs) {
        handoffIO.updateInputs(inputs);
    }
}