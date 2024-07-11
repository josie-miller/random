package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double setpointMeters = 0;
        public double elevatorHeightMeters = 0;
        public double elevatorVelMPS = 0;
        public double[] currentAmps = new double[] {};
        public double[] tempF = new double[] {};
    }

public default void updateInputs(ElevatorIOInputs inputs) {
}

public default void zeroSensor() {
}

public default void setMotionMagicSetpoint(double setpointMeters) {
}

public default void runElevator(double voltage) {

}

}