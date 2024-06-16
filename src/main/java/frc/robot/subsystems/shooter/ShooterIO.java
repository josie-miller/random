package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double current = 0.0;
        public double temp = 0.0;
        public double RPS = 0.0;
    }

    public void updateInputs(ShooterIOInputs inputs);

    public void setVelocity(double velocity, double ratio);

    public void setVoltage(double voltage);

}
