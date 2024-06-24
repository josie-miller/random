package frc.robot.subsystems.handoff;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Handoff extends SubsystemBase {
    private final HandoffIO handoffIO;
    private HandoffIOInputsAutoLogged inputs = new HandoffIOInputsAutoLogged();
    private double setpointVolts;

    public Handoff(HandoffIO handoffIO) {
        this.handoffIO = handoffIO;
        setpointVolts = 0.0;
    }
    public void Loop(){
        handoffIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    public Command runHandoff(double voltage) {
        setpointVolts = voltage;
        return this.run(() -> handoffIO.setVoltage(setpointVolts));
    }

    public Command runHandoff(double voltage, double time) {
        return this.run(() -> handoffIO.setVoltage(voltage))
        .withTimeout(time)
        .andThen(this.runOnce(() -> handoffIO.setVoltage(0)));

    }

    public double getStatorCurrent(){
        return inputs.current;
    }

    public void updateInputs(HandoffIO.HandoffIOInputs inputs) {
        handoffIO.updateInputs(inputs);
        inputs.setpointVolts = this.setpointVolts;
    }
}