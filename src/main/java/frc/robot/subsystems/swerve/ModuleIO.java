package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;
        public double driveDistanceMeters = 0.0;
        public double driveOutputPercent = 0.0;
        public double rawDriveRPS = 0.0;

        public double moduleAngleRads = 0.0;
        public double moduleAngleDegs = 0.0;
        public double rawAbsolutePositionRotations = 0.0;
        public double absolutePositionRadians = 0.0;
        public double absolutePositionDegrees = 0.0; 
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0; 
        public double turnTempCelcius = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){}

    public default void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {}

    public default void setDriveVoltage(double setVoltage) {}

    public default void steerVoltage(double voltage){}

    public default void setTurnAngle(double positionDegs) {}

    public default void setDriveBrakeMode(boolean enable) {}

    public default void setTurnBrakeMode(boolean enable) {}

    public default void resetToAbsolute() {}

}