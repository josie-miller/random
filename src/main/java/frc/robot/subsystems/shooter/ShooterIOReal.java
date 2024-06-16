package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.Conversions;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final StatusSignal<Double> current1;
    private final StatusSignal<Double> temp1;
    private final StatusSignal<Double> RPS1;
    private final StatusSignal<Double> position1;

    private VoltageOut shootRequestVoltage;
    private VelocityVoltage leftRequestVelocity;
    private VelocityVoltage rightRequestVelocity;

    public ShooterIOReal() {
        leftMotor = new TalonFX(Constants.canIDConstants.leftShooterMotor, "canivore");
        rightMotor = new TalonFX(Constants.canIDConstants.rightShooterMotor, "canivore");
        current1 = leftMotor.getStatorCurrent();
        temp1 = leftMotor.getDeviceTemp();
        RPS1 = leftMotor.getRotorVelocity();
        position1 = leftMotor.getPosition();

        shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
        leftRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
        rightRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

        var leftMotorConfigs = new TalonFXConfiguration();
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.shooterConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfigs.Slot0.kP = Constants.shooterConstants.kP;
        leftMotorConfigs.Slot0.kI = 0.0;
        leftMotorConfigs.Slot0.kD = Constants.shooterConstants.kD;
        leftMotorConfigs.Slot0.kS = Constants.shooterConstants.kS;
        leftMotorConfigs.Slot0.kV = Constants.shooterConstants.kV;
        leftMotorConfigs.Slot0.kA = Constants.shooterConstants.kA;

        var rightMotorConfigs = new TalonFXConfiguration();
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
            current1,
            temp1,
            RPS1,
            position1
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void setVelocity(double velocity, double ratio) {
        leftMotor.setControl(leftRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity, Constants.shooterConstants.wheelCircumferenceMeters, 1.0)));
        rightMotor.setControl(rightRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity * ratio, Constants.shooterConstants.wheelCircumferenceMeters, 1.0)));
    }

    @Override
    public void setVoltage(double voltage) {
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
        leftMotor.setControl(shootRequestVoltage.withOutput(voltage));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(current1, temp1, RPS1);
        inputs.current = current1.getValue();
        inputs.temp = temp1.getValue();
        inputs.RPS = RPS1.getValue();
    }
}