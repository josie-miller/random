package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Conversions;
import frc.robot.Constants.swerveConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;
    private final double CANcoderOffset;
    /* */
    private final InvertedValue driveInvert;
    private final InvertedValue steerInvert;
    private final SensorDirectionValue CANcoderInvert;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;
    private CANcoderConfigurator angleEncoderConfigurator;
    private TalonFXConfigurator driveConfigurator;
    private TalonFXConfigurator steerConfigurator;

    private final StatusSignal<Double> steerPos;
    private final StatusSignal<Double> drivePos;
    private final StatusSignal<Double> driveVelRPS;
    private final StatusSignal<Double> driveTemp;
    private final StatusSignal<Double> steerTemp;
    private final StatusSignal<Double> driveAmps;
    private final StatusSignal<Double> steerAmps;
    private final StatusSignal<Double> absolutePositionRotations;

    private PositionVoltage steerRequest;
    private VelocityVoltage velocityVoltageRequest;
    private VoltageOut driveVoltageRequest;
    private VoltageOut steerVoltageRequest;
/* 
    private final double drivekP = 0;
    private final double drivekD = 0;
    private final double drivekS = 0;
    private final double drivekV = 0;

    private final double steerkP = 8;
    private final double steerkD = 0;
    private final double steerkS = 0;
    private final double steerkV = 0;

    private final double voltage = 0;*/

    public SwerveModule(int driveID, int steerID, int CANcoderID, double CANcoderOffset, InvertedValue driveInvert, InvertedValue steerInvert, SensorDirectionValue CANcoderInvert) {
        driveMotor = new TalonFX(driveID, "canivore");
        steerMotor = new TalonFX(steerID, "canivore");
        angleEncoder = new CANcoder(CANcoderID, "canivore");
        this.CANcoderOffset = CANcoderOffset;
        this.driveInvert = driveInvert;
        this.steerInvert = steerInvert;
        this.CANcoderInvert = CANcoderInvert;
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();
        driveConfigurator = driveMotor.getConfigurator();
        steerConfigurator = steerMotor.getConfigurator();
        angleEncoderConfigurator = angleEncoder.getConfigurator();

        steerRequest = new PositionVoltage(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();

        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = driveInvert;
        driveMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
        driveMotorOutputConfigs.PeakReverseDutyCycle = -1.0;

        var driveFeedbackConfigs = driveConfigs.Feedback;
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setPosition(0);

        var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
        driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        driveCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.driveStatorCurrentLimit;

        var driveOpenLoopConfigs = driveConfigs.OpenLoopRamps;
        driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = swerveConstants.rampRate;

        var driveSlot0Configs = driveConfigs.Slot0;
         
         //swan carpet
        driveSlot0Configs.kP = 0.13995; // 0.13995
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = 0.0;
        driveSlot0Configs.kS = 0.011412; //  0.011412
        driveSlot0Configs.kV = 0.12125;// 0.12125
        driveSlot0Configs.kA = 0.042716; // 0.042716*/

        //field carpet
        /* 
        driveSlot0Configs.kP = 0.12006; // 0.13995
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = 0.0;
        driveSlot0Configs.kS = 0.21146; //  0.011412
        driveSlot0Configs.kV = 0.12209;// 0.12125
        driveSlot0Configs.kA = 0.013607; // 0.042716*/

        // STEER

        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = steerInvert;

        var steerFeedbackConfigs = steerConfigs.Feedback;
        steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // steerFeedbackConfigs.FeedbackRemoteSensorID = CANcoderID;
        // steerFeedbackConfigs.RotorToSensorRatio =
        // swerveConstants.moduleConstants.steerGearRatio;
        // steerFeedbackConfigs.SensorToMechanismRatio = 1.0;
        // steerFeedbackConfigs.FeedbackRotorOffset = 0;

        var steerSlot0Configs = steerConfigs.Slot0;
        steerSlot0Configs.kP = 10.309;
        steerSlot0Configs.kI = 0.0;
        steerSlot0Configs.kD = 0.11175;
        steerSlot0Configs.kS = 0.30895;
        steerSlot0Configs.kV = 0.12641;
        steerSlot0Configs.kA = 0.0016487;

        var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
        steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        steerCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.steerStatorCurrentLimit;

        // CANcoder
        var magnetSensorConfigs = angleEncoderConfigs.MagnetSensor;
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.MagnetOffset = 0;
        magnetSensorConfigs.SensorDirection = CANcoderInvert;

        driveConfigurator.apply(driveConfigs);
        steerConfigurator.apply(steerConfigs);
        angleEncoderConfigurator.apply(angleEncoderConfigs);

        steerPos = steerMotor.getRotorPosition();
        drivePos = driveMotor.getRotorPosition();
        driveVelRPS = driveMotor.getRotorVelocity();
        driveTemp = driveMotor.getDeviceTemp();
        steerTemp = steerMotor.getDeviceTemp();
        driveAmps = driveMotor.getStatorCurrent();
        steerAmps = steerMotor.getStatorCurrent();
        absolutePositionRotations = angleEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                steerPos,
                drivePos,
                driveVelRPS,
                driveTemp,
                steerTemp,
                driveAmps,
                steerAmps,
                absolutePositionRotations);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    public void setDesiredState(SwerveModuleState optimizedDesiredStates, boolean isOpenLoop) {
        if(isOpenLoop){
            double driveVoltage = optimizedDesiredStates.speedMetersPerSecond * 7 ;
            double angleDeg = optimizedDesiredStates.angle.getDegrees();

            setDriveVoltage(driveVoltage);
            setTurnAngle(angleDeg);
        }
        else if(!isOpenLoop){
            double driveVelocity = optimizedDesiredStates.speedMetersPerSecond;
            double angleDeg = optimizedDesiredStates.angle.getDegrees();

            setDriveVelocity(driveVelocity, true);
            setTurnAngle(angleDeg);
        }
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setControl(driveVoltageRequest.withOutput(volts));
    }

    public void steerVoltage(double volts) {
        steerMotor.setControl(steerVoltageRequest.withOutput(volts));
    }

    public void setTurnAngle(double angleDeg) {
        steerMotor.setControl(steerRequest.withPosition(
                Conversions.DegreesToRotations(angleDeg, swerveConstants.steerGearRatio)));
    }

    public void resetToAbsolute() {
        double absolutePositionRotations = angleEncoder.getAbsolutePosition().getValue() - CANcoderOffset;
        double absolutePositionSteerRotations = absolutePositionRotations * swerveConstants.steerGearRatio;
        steerMotor.setPosition(absolutePositionSteerRotations);
    }

    public void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {
        velocityVoltageRequest.Velocity = Conversions.MPStoRPS(velocityMetersPerSecond,
                swerveConstants.wheelCircumferenceMeters,
                swerveConstants.driveGearRatio);
        driveMotor.setControl(velocityVoltageRequest);
    }

}