package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLog; 

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

    public Command runIntake(double voltage) {
        setpointVolts = voltage;
        return this.run(() -> intakeIO.runIntake(setpointVolts));
    }

    public Command runIntake(double voltage, double time) {
        return this.run(() -> intakeIO.runIntake(voltage))
            .withTimeout(time)
            .andThen(this.runOnce(() -> intakeIO.runIntake(0)))
                .onlyIf(() -> atSetpoint());
    }
    public double getStatorCurrent(){
        return inputs.currentAmps;
    }

    public boolean atSetpoint() {
        return this.getStatorCurrent() > 38;
    }

    public void updateInputs(IntakeIO.IntakeIOInputs inputs) {
        intakeIO.updateInputs(inputs);
    }
}