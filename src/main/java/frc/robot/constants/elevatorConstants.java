package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class elevatorConstants {
    public static final InvertedValue leftMotorInvert = InvertedValue.CounterClockwise_Positive;

    public static final double gearRatio = 17;
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(5.51873699838);
    public static final double minHeightMeters = 0.0;
    public static final double maxHeightMeters = 0.45;

    public static final double minHeightInRotations = 0;
    public static final double maxHeightInRotations = 0.0;

    /* Current Limits */
    public static final double statorCurrentLimit = 80;

    /* PID Values*/
    public static final double kP = 8.413;
    public static final double kD = 0.0030141;
    public static final double kS = 0.058684;
    public static final double kV = 0.0044;
    public static final double kG = 0.0662029;

    /* MotionMagic Values */
    public static final double CruiseVelocityUp = 75;
    public static final double AccelerationUp = 150;
    public static final double Jerk = 10000;

    public static final double CruiseVelocityDown = 10;
    public static final double AccelerationDown = 20;

    public static final double ToleranceMeters = 0.01;

}
