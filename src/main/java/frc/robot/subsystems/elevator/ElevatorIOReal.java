package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final StatusSignal<Double> current1;
    private final StatusSignal<Double> temp1;
    private final StatusSignal<Double> RPS1;
    private final StatusSignal<Double> position1;

    public ElevatorIOReal() {
        leftMotor = new TalonFX(Constants.canIDConstants.elevatorMotor1, "canivore");
        rightMotor = new TalonFX(Constants.canIDConstants.elevatorMotor2, "canivore");

        current1 = leftMotor.getStatorCurrent();
        temp1 = leftMotor.getDeviceTemp();
        RPS1 = leftMotor.getRotorVelocity();
        position1 = leftMotor.getPosition();

        var leftMotorConfigs = new TalonFXConfiguration();
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.elevatorConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leftMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.elevatorConstants.CruiseVelocityUp;
        leftMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.elevatorConstants.AccelerationUp;
        leftMotorConfigs.Slot0.kP = Constants.elevatorConstants.kP;
        leftMotorConfigs.Slot0.kD = Constants.elevatorConstants.kD;
        leftMotorConfigs.Slot0.kS = Constants.elevatorConstants.kS;
        leftMotorConfigs.Slot0.kV = Constants.elevatorConstants.kV;
        leftMotorConfigs.Slot0.kG = Constants.elevatorConstants.kG;

        leftMotor.getConfigurator().apply(leftMotorConfigs);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

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
    public void setMotionMagicSetpoint(double setpointRotations) {
        leftMotor.setControl(new MotionMagicVoltage(setpointRotations));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(current1, temp1, RPS1, position1);
        inputs.current = current1.getValue();
        inputs.temperature = temp1.getValue();
        inputs.RPS = RPS1.getValue();
        inputs.position = position1.getValue();
    }

    @Override
    public void zeroSensorPosition() {
        leftMotor.setPosition(0);
    }

    @Override
    public double getPosition() {
        return position1.getValue();
    }

    @Override
    public void disableMotors() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }
}