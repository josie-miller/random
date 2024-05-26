package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public final class Constants {
    public static final class canIDConstants {
        public static final int intakeMotor = 11;
    }

    public static final class intakeConstants {
        public static final double statorCurrentLimit = 50.0; 
        public static final InvertedValue intakeInvert = InvertedValue.Clockwise_Positive;
    }
}
