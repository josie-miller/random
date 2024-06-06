package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import java.lang.Math;

public final class Constants {

    public static final class canIDConstants {
        public static final int intakeMotor = 15;
        public static final int elevatorMotor1 = 13;
        public static final int elevatorMotor2 = 14;
        public static final int handoffMotor = 16;
        public static final int leftShooterMotor = 17;
        public static final int rightShooterMotor = 18;

    }

    public static final class intakeConstants {
        public static final double statorCurrentLimit = 70.0;
        public static final InvertedValue intakeInvert = InvertedValue.CounterClockwise_Positive;
    }

    public static final class elevatorConstants {

        // copied straight from elevator 

        public static final InvertedValue leftMotorInvert = InvertedValue.Clockwise_Positive; 
        public static final double gearRatio = 17;
        public static final double wheelCircumferenceMeters = Units.inchesToMeters(5.51873699838);
        public static final double minHeightMeters = 0.0;
        public static final double maxHeightMeters = 0.45;

        public static final double minHeightInRotations = 0;
        public static final double maxHeightInRotations = 0.0;
        // Current Limit
        public static final double statorCurrentLimit = 80;

        // PID Values
        public static final double kP = 8.413;
        public static final double kD = 0.0030141;
        public static final double kS = 0.058684;
        public static final double kV = 0.0044;
        public static final double kG = 0.0662029;

        // MotionMagic Values
        public static final double CruiseVelocityUp = 75;
        public static final double AccelerationUp = 150;
        public static final double Jerk = 10000;

        public static final double CruiseVelocityDown = 10;
        public static final double AccelerationDown = 20;

        public static final double ToleranceMeters = 0.01;

    }
    public static final class handoffConstants {
        public static final InvertedValue handoffInvert = InvertedValue.Clockwise_Positive;
        public static final double statorCurrentLimit = 70.0;
    }
    public static final class shooterConstants {
        public static double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;
   
        public static final InvertedValue leftShooterInvert = InvertedValue.Clockwise_Positive;

    
        public static final double statorCurrentLimit = 0.0;

   
        public static final double kP = 0.068419;
        public static final double kD = 0.0;
        public static final double kS = 0.16488;
        public static final double kV = 0.11167;
        public static final double kA = 0.0077173;
    }
}
