package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {

        public double appliedVolts = 0.0;
        public double setpointVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempF = 0.0;
        public double intakeRPS = 0.0;

    }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void runIntake(double voltage) {
    }

}