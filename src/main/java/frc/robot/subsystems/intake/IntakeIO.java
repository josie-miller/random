package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double appliedVolts = 0.0;
        public double setpointVolts = 0.0;
        public double current = 0.0;
        public double tempF = 0.0;
        public double intakeRPS = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public Command setOutput(double volts);
}
