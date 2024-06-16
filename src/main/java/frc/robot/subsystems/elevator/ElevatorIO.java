package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double current = 0.0;
        public double temperature = 0.0;
        public double RPS = 0.0;
        public double position = 0.0;
    }

    public void updateInputs(ElevatorIOInputs inputs);

    public void setMotionMagicSetpoint(double setpointRotations);

    public void zeroSensorPosition();

    public double getPosition();

    public void disableMotors();
}
