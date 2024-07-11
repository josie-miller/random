package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import frc.robot.Conversions;
import frc.robot.Constants.swerveConstants;

public class ModuleIOReal implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;
    private final double CANcoderOffset;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;

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

    double drivekP = swerveConstants.drivekP;
    double drivekD = swerveConstants.drivekD;
    double drivekS = swerveConstants.drivekS;
    double drivekV = swerveConstants.drivekV;

    double steerkP = swerveConstants.steerkP;
    double steerkD = swerveConstants.steerkD;
    double steerkS = swerveConstants.steerkS;
    double steerkV = swerveConstants.steerkV;

    double voltage = 0;

    public ModuleIOReal(int driveID, int steerID, int CANcoderID, double CANcoderOffset, InvertedValue driveInvert,
            InvertedValue steerInvert, SensorDirectionValue CANcoderInvert) {
        driveMotor = new TalonFX(driveID, "canivore");
        steerMotor = new TalonFX(steerID, "canivore");
        angleEncoder = new CANcoder(CANcoderID, "canivore");
        this.CANcoderOffset = CANcoderOffset;
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();

        steerRequest = new PositionVoltage(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();

        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfigs.MotorOutput.Inverted = driveInvert;
        driveConfigs.MotorOutput.PeakForwardDutyCycle = 1.0;
        driveConfigs.MotorOutput.PeakReverseDutyCycle = -1.0;
        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setPosition(0);

        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.StatorCurrentLimit = swerveConstants.driveStatorCurrentLimit;
        driveConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = swerveConstants.rampRate;

        driveConfigs.Slot0.kP = 0.12006; 
        driveConfigs.Slot0.kI = 0.0;
        driveConfigs.Slot0.kD = 0.0;
        driveConfigs.Slot0.kS = 0.21146;
        driveConfigs.Slot0.kV = 0.12209;
        driveConfigs.Slot0.kA = 0.013607;

        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfigs.MotorOutput.Inverted = steerInvert;

        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfigs.Feedback.FeedbackRemoteSensorID = CANcoderID;
        steerConfigs.Feedback.RotorToSensorRatio = swerveConstants.steerGearRatio;

        steerConfigs.Slot0.kP = 10.309;
        steerConfigs.Slot0.kI = 0.0;
        steerConfigs.Slot0.kD = 0.11175;
        steerConfigs.Slot0.kS = 0.30895;
        steerConfigs.Slot0.kV = 0.12641;
        steerConfigs.Slot0.kA = 0.0016487;

        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.StatorCurrentLimit = swerveConstants.steerStatorCurrentLimit;

        angleEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        angleEncoderConfigs.MagnetSensor.MagnetOffset = 0;
        angleEncoderConfigs.MagnetSensor.SensorDirection = CANcoderInvert;

        driveMotor.getConfigurator().apply(driveConfigs);
        steerMotor.getConfigurator().apply(steerConfigs);
        angleEncoder.getConfigurator().apply(angleEncoderConfigs);

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
                steerAmps);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                steerPos,
                drivePos,
                driveVelRPS,
                driveTemp,
                steerTemp,
                driveAmps,
                steerAmps);

        inputs.driveVelocityMetersPerSec = Conversions.RPStoMPS(driveVelRPS.getValue(), swerveConstants.wheelCircumferenceMeters, swerveConstants.driveGearRatio);
        inputs.driveAppliedVolts = driveVoltageRequest.Output;
        inputs.driveCurrentAmps = driveAmps.getValue();
        inputs.driveTempCelcius = driveTemp.getValue();
        inputs.driveDistanceMeters = Conversions.RotationsToMeters(drivePos.getValue(), swerveConstants.wheelCircumferenceMeters, swerveConstants.driveGearRatio);
        inputs.driveOutputPercent = driveMotor.get();
        inputs.rawDriveRPS = driveVelRPS.getValue();

        inputs.moduleAngleRads = Units.degreesToRadians(Conversions.RotationsToDegrees(steerPos.getValue(), swerveConstants.steerGearRatio));
        inputs.moduleAngleDegs = Conversions.RotationsToDegrees(steerPos.getValue(), swerveConstants.steerGearRatio);
        inputs.rawAbsolutePositionRotations = absolutePositionRotations.getValue();
        inputs.absolutePositionRadians = absolutePositionRotations.getValue() * 2 * Math.PI;
        inputs.absolutePositionDegrees = absolutePositionRotations.getValue() * 360;
        inputs.turnAppliedVolts = steerVoltageRequest.Output;
        inputs.turnCurrentAmps = steerAmps.getValue();
        inputs.turnTempCelcius = steerTemp.getValue();
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
        velocityVoltageRequest.Velocity = Conversions.MPStoRPS(velocityMetersPerSecond, swerveConstants.wheelCircumferenceMeters, swerveConstants.driveGearRatio);
        driveMotor.setControl(velocityVoltageRequest);
    }

}