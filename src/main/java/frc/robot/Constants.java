package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.lang.Math;

public final class Constants {

    public static final boolean tuningMode = true;

    public static final class canIDConstants {

        public static final int pigeon = 0;
        public static final int[] driveMotor = { 1, 2, 3, 4 };
        public static final int[] steerMotor = { 5, 6, 7, 8 };
        public static final int[] CANcoder = { 9, 10, 11, 12 };


        public static final int elevatorMotor1 = 13;
        public static final int elevatorMotor2 = 14;
        public static final int intakeMotor = 15;
        public static final int handoffMotor = 16;
        public static final int leftShooterMotor = 17;
        public static final int rightShooterMotor = 18;
        public static final int otbIntakePivotMotor = 19;
        public static final int otbIntakeMotor = 20;
    }

    public static final class swerveConstants {
        /* Inverts FL, FR, BL, BR */
        public static final InvertedValue[] driveMotorInverts = {InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final InvertedValue[] steerMotorInverts = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final SensorDirectionValue[] CANcoderInverts = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};
        public static final double[] CANcoderOffsets = {0.4812016, 0.8261721, 0.611572, 0.819092}; //degrees or rotations 178.418, 204.17, 220.254, 358.154
 // 0.518311, 0.271240, 0.386230, 0.173340
        /* CANcoder Offset FL, FR, BL, BR */
        
        /* Gear Ratios */
        public static final double driveGearRatio = 6.55;
        public static final double steerGearRatio = 10.28;

        /* Max Speeds */
        public static final double maxSpeed =2; 
        public static final double maxAngularVelocity = 2.5; 
        
        /* Current Limits */
        public static final double driveStatorCurrentLimit = 80;
        public static final double steerStatorCurrentLimit = 50;

        /* Ramp Rate */
        public static final double rampRate = 0.02;
        
        /* PID Values */
        public static final double drivekP = 0.0;
        public static final double drivekD = 0.0;
        public static final double drivekS = 0.0;
        public static final double drivekV = 0.0;

        public static final double anglekP = 0.0;
        public static final double anglekD = 0.0;
        public static final double anglekS = 0.0;
        public static final double anglekV = 0.0;

        public static final double steerkP = 8.0;
        public static final double steerkD = 0.0;
        public static final double steerkS = 0.0;
        public static final double steerkV = 0.0;

        /* Wheel Circumference */
        public static final double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;

        /* Drivetrain Constants */
        public static final double robotLength = Units.inchesToMeters(28);
        public static final double robotWidth = Units.inchesToMeters(28.5);

        public static final double trackWidth = 0.60325;
        public static final double wheelBase = 0.59055;

        /* Swerve Kinematics */
        public static final Translation2d FL = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d FR = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

        /* Extra */

        public static final double velocityToVoltageScaler = 7.0;

    }

    public static final class intakeConstants {
        public static final double statorCurrentLimit = 70.0;
        public static final InvertedValue intakeInvert = InvertedValue.CounterClockwise_Positive;
    }

    public static final class elevatorConstants {

        // copied straight from elevator 

        public static final InvertedValue leftMotorInvert = InvertedValue.CounterClockwise_Positive; 
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

    
        public static final double statorCurrentLimit = 70.0;

   
        public static final double kP = 0.068419;
        public static final double kD = 0.0;
        public static final double kS = 0.16488;
        public static final double kV = 0.11167;
        public static final double kA = 0.0077173;
    }
    public static final class otbIntakeConstants {
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

    public static final class commandConstants {
        public static final double handoffShooterVoltage = 3;
        public static final double handoffIntakeVoltage = handoffShooterVoltage; // 1 volt
        public static final double intakeVoltage = 4;
        public static final double outakeVoltage = -3.5;
        public static final double AMPShooterVelocity = 3.2;
        public static final double shootMidVelocity = 20;
        public static final double shootRightVelocity = 15;
        public static final double shootLeftVelocity = 20;
        public static final double midShootRatio = 0.7;
        public static final double rightShootRatio = 1.7;
        public static final double leftShootRatio = 0.5;
        public static final double maxHeight = 0.45;
        public static final double minHeight = 0.0;
        public static final double midShootTime = 1.2;
        public static final double rightShootTime = 0.7;
        public static final double leftShootTime = 1;
        public static final double AMPShootTime = 1;
        public static final double AMPShootRatio = 1;
        public static final double leftShortTime = 0.4;
        public static final double rightShortTime = 0.4;
        public static final double midShortTime = 0.8;
        public static final double AMPShortTime = 0.4;
        public static final double floorDegrees = 138;
        public static final double otbVoltage = 4;
        public static final double restingDegrees = 21;

    }
}
