package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.AutoLog;

public interface OtbIntakeIO {
    @AutoLog
    public static class OtbIntakeIOInputs {
        public double intakeCurrent = 0.0;
        public double intakeTemp = 0.0;
        public double intakeRPS = 0.0;
        public double setpointVolts = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotTemp = 0.0;
        public double pivotRPS = 0.0;
        public double pivotSetpointDeg = 0.0;
    }

    public void updateInputs(OtbIntakeIOInputs inputs);

    public void setPivotVoltage(double voltage);

    public void setPivotPosition(double angleDegrees);

    public void setIntakeVoltage(double voltage);

    public void zeroPosition();
}