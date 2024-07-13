package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double setpointVolts;

    public Intake(IntakeIO intakeIO) {
        this.intakeIO = intakeIO;
        setpointVolts = 0.0;
    }

    public void periodic(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void runIntake(double voltage) {
        setpointVolts = voltage;
        intakeIO.runIntake(setpointVolts);
    }

    public double getStatorCurrent(){
        return inputs.currentAmps;
    }

    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        intakeIO.updateInputs(inputs);
    }
}