package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

import java.lang.Math;

public class shooterConstants {
    public static double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;
    /* Motor Inverts */
    public static final InvertedValue leftShooterInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double statorCurrentLimit = 0.0;

    /* PID values */
    public static final double kP = 0.068419;
    public static final double kD = 0.0;
    public static final double kS = 0.16488;
    public static final double kV = 0.11167;
    public static final double kA = 0.0077173;

}