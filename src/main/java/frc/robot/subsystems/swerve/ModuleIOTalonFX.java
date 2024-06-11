package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Conversions;

public class ModuleIOTalonFX implements ModuleIO {
        private final TalonFX driveMotor;
        private final TalonFX steerMotor;
        private final CANcoder angleEncoder;
        private final double CANcoderOffset;
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

        public ModuleIOTalonFX(int driveID, int steerID, int CANcoderID, double CANcoderOffset, InvertedValue driveInvert, InvertedValue steerInvert, SensorDirectionValue CANcoderInvert) {
            driveMotor = new TalonFX(driveID, "canivore");
            steerMotor = new TalonFX(steerID, "canivore");
            angleEncoder = new CANcoder(CANcoderID, "canivore");
            this.CANcoderOffset = CANcoderOffset;
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
            driveCurrentLimitConfigs.StatorCurrentLimit = Constants.swerveConstants.driveStatorCurrentLimit;

            var driveOpenLoopConfigs = driveConfigs.OpenLoopRamps;
            driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = Constants.swerveConstants.rampRate;

            var driveSlot0Configs = driveConfigs.Slot0;
            //swan carp
            driveSlot0Configs.kP = 0.13995; 
            driveSlot0Configs.kI = 0.0;
            driveSlot0Configs.kD = 0.0;
            driveSlot0Configs.kS = 0.011412; 
            driveSlot0Configs.kV = 0.12125;
            driveSlot0Configs.kA = 0.042716; 

            //Steer

            var steerMotorOutputConfigs = steerConfigs.MotorOutput;
            steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
            steerMotorOutputConfigs.Inverted = steerInvert;
            var steerFeedbackConfigs = steerConfigs.Feedback;
            steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

            var steerSlot0Configs = steerConfigs.Slot0;
            steerSlot0Configs.kP = 10.309;
            steerSlot0Configs.kI = 0.0;
            steerSlot0Configs.kD = 0.11175;
            steerSlot0Configs.kS = 0.30895;
            steerSlot0Configs.kV = 0.12641;
            steerSlot0Configs.kA = 0.0016487;

            var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
            steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
            steerCurrentLimitConfigs.StatorCurrentLimit = Constants.swerveConstants.steerStatorCurrentLimit;

            //CANcoder

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
    
            inputs.driveVelocityMetersPerSec = Conversions.RPStoMPS(driveVelRPS.getValue(), Constants.swerveConstants.wheelCircumferenceMeters, Constants.swerveConstants.driveGearRatio);
            inputs.driveAppliedVolts = driveVoltageRequest.Output;
            inputs.driveCurrentAmps = driveAmps.getValue();
            inputs.driveTempCelcius = driveTemp.getValue();
            inputs.driveDistanceMeters = Conversions.RotationsToMeters(drivePos.getValue(), Constants.swerveConstants.wheelCircumferenceMeters, Constants.swerveConstants.driveGearRatio);
            inputs.driveOutputPercent = driveMotor.get();
            inputs.rawDriveRPS = driveVelRPS.getValue();
    
            inputs.moduleAngleRads = Units.degreesToRadians(Conversions.RotationsToDegrees(steerPos.getValue(), Constants.swerveConstants.steerGearRatio));
            inputs.moduleAngleDegs = Conversions.RotationsToDegrees(steerPos.getValue(), Constants.swerveConstants.steerGearRatio);
            inputs.rawAbsolutePositionRotations = absolutePositionRotations.getValue();
            inputs.absolutePositionRadians = absolutePositionRotations.getValue() * 2 * Math.PI;
            inputs.absolutePositionDegrees = absolutePositionRotations.getValue() * 360;
            inputs.turnAppliedVolts = steerVoltageRequest.Output;
            inputs.turnCurrentAmps = steerAmps.getValue();
            inputs.turnTempCelcius = steerTemp.getValue();
        }

        public void setDesiredState(SwerveModuleState optimizedDesiredStates, boolean isOpenLoop) {
            if(isOpenLoop){
                double driveVoltage = optimizedDesiredStates.speedMetersPerSecond * Constants.swerveConstants.velocityToVoltageScaler ;
                double angleDeg = optimizedDesiredStates.angle.getDegrees();
    
                setDriveVoltage(driveVoltage);
                setTurnAngle(angleDeg);
            }
            else if(!isOpenLoop){
                double driveVelocity = optimizedDesiredStates.speedMetersPerSecond;
                double angleDeg = optimizedDesiredStates.angle.getDegrees();
    
                setDriveVelocity(driveVelocity);
                setTurnAngle(angleDeg);
            }
        }

        public void setDriveVelocity(double velocityMetersPerSecond) {
            velocityVoltageRequest.Velocity = Conversions.MPStoRPS(velocityMetersPerSecond,
                    Constants.swerveConstants.wheelCircumferenceMeters,
                    Constants.swerveConstants.driveGearRatio);
            driveMotor.setControl(velocityVoltageRequest);
        }

        public void setDriveVoltage(double volts) {
            driveMotor.setControl(driveVoltageRequest.withOutput(volts));
        }

        public void steerVoltage(double volts) {
            steerMotor.setControl(steerVoltageRequest.withOutput(volts));
        }

        public void setTurnAngle(double angleDeg) {
            steerMotor.setControl(steerRequest.withPosition(
                    Conversions.DegreesToRotations(angleDeg, Constants.swerveConstants.steerGearRatio)));
        }

        public void resetToAbsolute() {
            double absolutePositionRotations = angleEncoder.getAbsolutePosition().getValue() - CANcoderOffset;
            double absolutePositionSteerRotations = absolutePositionRotations * Constants.swerveConstants.steerGearRatio;
            steerMotor.setPosition(absolutePositionSteerRotations);
        }
    


    




    }
