package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.Conversions;

public class ShooterIOReal implements ShooterIO{
    private final TalonFX leftMotor = new TalonFX(Constants.canIDConstants.leftShooterMotor, "canivore");
    private final TalonFX rightMotor = new TalonFX(Constants.canIDConstants.rightShooterMotor, "canivore");
    private TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
    private TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

    private final StatusSignal<Double> leftShooterCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Double> rightShooterCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Double> leftShooterTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Double> rightShooterTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<Double> leftShooterSpeedRPS = leftMotor.getRotorVelocity();
    private final StatusSignal<Double> rightShooterSpeedRPS = rightMotor.getRotorVelocity();

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    
    private double leftShooterSetpointMPS;
    private double rightShooterSetpointMPS;

    public ShooterIOReal() {

        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.shooterConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfigs.Slot0.kP = Constants.shooterConstants.kP;
        leftMotorConfigs.Slot0.kI = 0.0;
        leftMotorConfigs.Slot0.kD = Constants.shooterConstants.kD;
        leftMotorConfigs.Slot0.kS = Constants.shooterConstants.kS;
        leftMotorConfigs.Slot0.kV = Constants.shooterConstants.kV;
        leftMotorConfigs.Slot0.kA = Constants.shooterConstants.kA;

        rightMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.shooterConstants.statorCurrentLimit;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfigs.Slot0.kP = Constants.shooterConstants.kP;
        rightMotorConfigs.Slot0.kI = 0.0;
        rightMotorConfigs.Slot0.kD = Constants.shooterConstants.kD;
        rightMotorConfigs.Slot0.kS = Constants.shooterConstants.kS;
        rightMotorConfigs.Slot0.kV = Constants.shooterConstants.kV;
        rightMotorConfigs.Slot0.kA = Constants.shooterConstants.kA;

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }   

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS
        );

        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.currentAmps = new double[] { leftShooterCurrent.getValue(),
                rightShooterCurrent.getValue() };
        inputs.tempF = new double[] { leftShooterTemp.getValue(),
                rightShooterTemp.getValue() };
        inputs.shooterVelMPS = new double[] {Conversions.RPStoMPS(leftShooterSpeedRPS.getValue(), Constants.shooterConstants.wheelCircumferenceMeters, 1), Conversions.RPStoMPS(rightShooterSpeedRPS.getValue(), Constants.shooterConstants.wheelCircumferenceMeters, 1)};
        inputs.shooterSetpointsMPS = new double[] {leftShooterSetpointMPS, rightShooterSetpointMPS};
    }

    public void setVelocity(double velocity, double ratio) {
        leftShooterSetpointMPS = velocity;
        rightShooterSetpointMPS = velocity * ratio;
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity, Constants.shooterConstants.wheelCircumferenceMeters, 1)));
        rightMotor.setControl(rightShootRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity * ratio, Constants.shooterConstants.wheelCircumferenceMeters, 1)));

    }

    public void zeroVelocity(){
        leftShooterSetpointMPS = 0;
        rightShooterSetpointMPS = 0;
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(0));
        rightMotor.setControl(rightShootRequestVelocity.withVelocity(0));
    }

}
