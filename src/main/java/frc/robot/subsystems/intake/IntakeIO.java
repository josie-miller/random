package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double appliedVolts = 0.0;
        public double current = 0.0;
        public double tempF = 0.0;
        public double intakeRPS = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void setOutput(double volts);
}
