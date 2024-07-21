package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.Conversions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final StatusSignal<Double> current1;
    private final StatusSignal<Double> temp1;
    private final StatusSignal<Double> RPS1;
    private final StatusSignal<Double> position1;
    private double setpointMeters;

    public Elevator() {
        leftMotor = new TalonFX(Constants.canIDConstants.elevatorMotor1, "canivore");
        rightMotor = new TalonFX(Constants.canIDConstants.elevatorMotor2, "canivore");

        current1 = leftMotor.getStatorCurrent();
        temp1 = leftMotor.getDeviceTemp();
        RPS1 = leftMotor.getRotorVelocity();
        position1 = leftMotor.getPosition();

        var leftMotorConfigs = new TalonFXConfiguration();
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.elevatorConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.Inverted = Constants.elevatorConstants.leftMotorInvert;
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

        setpointMeters = 0;
    }

    public void setSetpoint(double setpointMeters) {
        this.setpointMeters = setpointMeters;
        double setpointRotations = Conversions.metersToRotations(setpointMeters, Constants.elevatorConstants.wheelCircumferenceMeters, Constants.elevatorConstants.gearRatio);
        leftMotor.setControl(new MotionMagicVoltage(setpointRotations));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current1, temp1, RPS1, position1);
        SmartDashboard.putNumber("Elevator Position", position1.getValue());
        SmartDashboard.putNumber("Elevator Current", current1.getValue());
        SmartDashboard.putNumber("Elevator Temperature", temp1.getValue());
        SmartDashboard.putNumber("Elevator Speed (RPS)", RPS1.getValue());
    }

    public void zeroSensor() {
        leftMotor.setPosition(0);
    }

    public boolean atSetpoint() {
        double currentPosMeters = Conversions.rotationsToMeters(position1.getValue(), Constants.elevatorConstants.wheelCircumferenceMeters, Constants.elevatorConstants.gearRatio);
        return Math.abs(currentPosMeters - setpointMeters) < Constants.elevatorConstants.ToleranceMeters; 
    }

    public void disable() {
        leftMotor.setVoltage(0);
        rightMotor.setVoltage(0);
    }
}
