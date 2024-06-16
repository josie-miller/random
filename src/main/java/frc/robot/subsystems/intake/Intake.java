package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private double setpointVolts;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
        setpointVolts = 0.0;
    }

    public void runIntake(double voltage) {
        setpointVolts = voltage;
        intakeIO.setOutput(voltage);
    }

    public Double getStatorCurrent(){
        return intakeIO.getStatorCurrent();
    }

    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        intakeIO.updateInputs(inputs);
        inputs.setpointVolts = this.setpointVolts;
    }
}
