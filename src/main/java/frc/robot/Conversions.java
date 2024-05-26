package frc.robot;

public class Conversions {
    public static double metersToRotations(double meters, double wheelCircumference, double gearRatio) {
        return (meters / wheelCircumference) * gearRatio;
    }

    public static double rotationsToMeters(double rotations, double wheelCircumference, double gearRatio) {
        return (rotations / gearRatio) * wheelCircumference;
    }
}
