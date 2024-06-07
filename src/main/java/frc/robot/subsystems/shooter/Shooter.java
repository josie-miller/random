package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;

public class Shooter extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final StatusSignal<Double> current1;
    private final StatusSignal<Double> temp1;
    private final StatusSignal<Double> RPS1;
    private final StatusSignal<Double> position1;
    private double leftsetpointMPS;
    private double rightsetpointMPS;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

    public Shooter() {
        leftMotor = new TalonFX(Constants.canIDConstants.leftShooterMotor, "canivore");
        rightMotor = new TalonFX(Constants.canIDConstants.rightShooterMotor, "canivore");
        current1 = leftMotor.getStatorCurrent();
        temp1 = leftMotor.getDeviceTemp();
        RPS1 = leftMotor.getRotorVelocity();
        position1 = leftMotor.getPosition();

        

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

        leftsetpointMPS = 0;
        rightsetpointMPS = 0;

    }

    public void runShooter(double velocity, double ratio) {
        leftsetpointMPS = velocity;
        rightsetpointMPS = velocity * ratio;
        leftMotor.setControl(leftRequestVelocity.withVelocity(Conversions.MPStoRPS(velocity, Constants.shooterConstants.wheelCircumferenceMeters, 1.0)));

    }

    public void setVoltage(double voltage) {
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
        leftMotor.setControl(shootRequestVoltage.withOutput(voltage));
        
    }

    
}
