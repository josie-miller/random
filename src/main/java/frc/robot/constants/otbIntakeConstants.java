package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class otbIntakeConstants {
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue intakeInvert = InvertedValue.CounterClockwise_Positive;

    public static final double pivotCurrentLimit = 50;
    public static final double intakeCurrentLimit = 50;

    public static final double gearRatio = 23.625;

    public static final double kP = 6.5; // 34.311
    public static final double kI = 0.0;
    public static final double kD = 0; //1.253
    public static final double kS = 0.169; //0.169
    public static final double kV = 0.0649; // 0.0649
    public static final double kA = 0.0246; //0.024675
    public static final double kG = 0.0301; //0.030166

    public static final double CruiseVelocity = 60;
    public static final double Acceleration = 120;
    public static final double Jerk = 10000;

}