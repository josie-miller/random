package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Conversions;
import frc.robot.Constants;

public class Module {
    private final Rotation2d angleOffset;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;

    private final PositionVoltage steerRequest;
    private final VelocityVoltage velocityVoltageRequest;
    private final VoltageOut driveVoltageRequest;
    
    public Module(int moduleNumber) {
        this.angleOffset = Rotation2d.fromRotations(Constants.swerveConstants.CANcoderOffsets[moduleNumber]);

        driveMotor = new TalonFX(Constants.canIDConstants.driveMotor[moduleNumber], "canivore");
        steerMotor = new TalonFX(Constants.canIDConstants.steerMotor[moduleNumber], "canivore");
        angleEncoder = new CANcoder(Constants.canIDConstants.CANcoder[moduleNumber], "canivore");

        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        CANcoderConfiguration angleEncoderConfigs = new CANcoderConfiguration();

        //drive configuration
        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = Constants.swerveConstants.driveMotorInverts[moduleNumber];
        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.StatorCurrentLimit = Constants.swerveConstants.driveStatorCurrentLimit;
        driveConfigs.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.swerveConstants.rampRate;
        driveConfigs.Slot0.kP = Constants.swerveConstants.drivekP;
        driveConfigs.Slot0.kD = Constants.swerveConstants.drivekD;
        driveConfigs.Slot0.kS = Constants.swerveConstants.drivekS;
        driveConfigs.Slot0.kV = Constants.swerveConstants.drivekV;

        //steer configuration
        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = Constants.swerveConstants.steerMotorInverts[moduleNumber];
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.StatorCurrentLimit = Constants.swerveConstants.steerStatorCurrentLimit;
        steerConfigs.Slot0.kP = Constants.swerveConstants.steerkP;
        steerConfigs.Slot0.kD = Constants.swerveConstants.steerkD;
        steerConfigs.Slot0.kS = Constants.swerveConstants.steerkS;
        steerConfigs.Slot0.kV = Constants.swerveConstants.steerkV;

        //encoder

        angleEncoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        angleEncoderConfigs.MagnetSensor.MagnetOffset = 0;
        angleEncoderConfigs.MagnetSensor.SensorDirection = Constants.swerveConstants.CANcoderInverts[moduleNumber];
        driveMotor.getConfigurator().apply(driveConfigs);
        steerMotor.getConfigurator().apply(steerConfigs);
        angleEncoder.getConfigurator().apply(angleEncoderConfigs);

        //control requests
        steerRequest = new PositionVoltage(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        //sensors
        driveMotor.setPosition(0);
        resetToAbsolute();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        steerMotor.setControl(steerRequest.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveVoltageRequest.Output = desiredState.speedMetersPerSecond / Constants.swerveConstants.maxSpeed;
            driveMotor.setControl(driveVoltageRequest);
        } else {
            velocityVoltageRequest.Velocity = Conversions.MPStoRPS(desiredState.speedMetersPerSecond, Constants.swerveConstants.wheelCircumferenceMeters, Constants.swerveConstants.driveGearRatio);
            driveMotor.setControl(velocityVoltageRequest);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        steerMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPStoMPS(driveMotor.getVelocity().getValue(), Constants.swerveConstants.wheelCircumferenceMeters, Constants.swerveConstants.driveGearRatio),
                Rotation2d.fromRotations(steerMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.RotationsToMeters(driveMotor.getPosition().getValue(), Constants.swerveConstants.wheelCircumferenceMeters, Constants.swerveConstants.driveGearRatio),
                Rotation2d.fromRotations(steerMotor.getPosition().getValue())
        );
    }
}
