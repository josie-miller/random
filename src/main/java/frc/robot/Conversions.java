package frc.robot;

public class Conversions {
    public static double metersToRotations(double meters, double wheelCircumference, double gearRatio) {
        return (meters / wheelCircumference) * gearRatio;
    }

    public static double rotationsToMeters(double rotations, double wheelCircumference, double gearRatio) {
        return (rotations / gearRatio) * wheelCircumference;
    }
    public static double RPMtoMPS(double RPM, double circumference, double gearRatio){
        double wheelMPS = (RPM * circumference) / 60.0;
        return wheelMPS;
    }

    public static double MPStoRPM(double MPS, double circumference, double gearRatio){
        double wheelRPM = (MPS * 60.0) / circumference;
        return wheelRPM;
    }

    public static double MPStoRPS(double MPS, double circumference, double gearRatio){
        double wheelRPS = MPS * gearRatio / circumference;
        return wheelRPS;
    }

    public static double RPStoMPS(double RPS, double circumference, double gearRatio){
        double wheelMPS = (RPS * circumference)/gearRatio;
        return wheelMPS;
    }
    public static double RotationsToDegrees(double RPM, double gearRatio){
        return RPM * (360.0 / (gearRatio));
    }
    
    public static double DegreesToRotations(double degrees, double gearRatio){
        return degrees / (360.0 / (gearRatio));
    }
    public static double RotationsToMeters(double Rotations, double circumference, double gearRatio){
        return Rotations * (circumference / (gearRatio));
    }
}
