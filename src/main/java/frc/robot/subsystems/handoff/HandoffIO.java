package frc.robot.subsystems.handoff;

import org.littletonrobotics.junction.AutoLog;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {
        public double current = 0.0;
        public double appliedVolts = 0.0;
        public double setpointVolts = 0.0;
        public double temp = 0.0;
        public double RPS = 0.0;
    }

    public void updateInputs(HandoffIOInputs inputs);

    public void setVoltage(double volts);
}
