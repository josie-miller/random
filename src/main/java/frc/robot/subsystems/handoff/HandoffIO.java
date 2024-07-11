package frc.robot.subsystems.handoff;

import org.littletonrobotics.junction.AutoLog;

public interface HandoffIO {
    @AutoLog
    public static class HandoffIOInputs {

        public double appliedVolts = 0.0;
        public double setpointVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempF = 0.0;
        public double handoffRPS = 0.0;

    }

    public default void updateInputs(HandoffIOInputs inputs) {
    }

    public default void runHandoff(double volts) {
    }



}