package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.AutoLog;

public interface OtbIntakeIO {
    @AutoLog
    public static class OtbIntakeIOInputs {
        public double intakeCurrent = 0.0;
        public double intakeTemp = 0.0;
        public double intakeRPS = 0.0;
        public double setpointVolts = 0.0;
    }

    public void updateInputs(OtbIntakeIOInputs inputs);

    public void setIntakeVoltage(double voltage);

}