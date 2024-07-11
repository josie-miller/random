package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.commons.Conversions;
import frc.robot.constants.canIDConstants;
import frc.robot.constants.elevatorConstants;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX leftMotor = new TalonFX(canIDConstants.leftElevatorMotor, "canivore");
    private final TalonFX rightMotor = new TalonFX(canIDConstants.rightElevatorMotor, "canivore");
    private final TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true); 
    private VoltageOut voltageOutRequest =  new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Double> current1 = leftMotor.getStatorCurrent();
    private final StatusSignal<Double> current2 = rightMotor.getStatorCurrent();
    private final StatusSignal<Double> temp1 = leftMotor.getDeviceTemp();
    private final StatusSignal<Double> temp2 = rightMotor.getDeviceTemp();
    private final StatusSignal<Double> RPS1 = leftMotor.getRotorVelocity();
    private final StatusSignal<Double> position1 = leftMotor.getPosition();
    private double setpointMeters;

    public ElevatorIOReal() {
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftMotorConfigs.MotorOutput.Inverted = elevatorConstants.leftMotorInvert;
        leftMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = elevatorConstants.CruiseVelocityUp;
        leftMotorConfigs.MotionMagic.MotionMagicAcceleration = elevatorConstants.AccelerationUp;
        leftMotorConfigs.MotionMagic.MotionMagicJerk = elevatorConstants.Jerk;

        leftMotorConfigs.Slot0.kP = elevatorConstants.kP;
        leftMotorConfigs.Slot0.kD = elevatorConstants.kD;
        leftMotorConfigs.Slot0.kS = elevatorConstants.kS;
        leftMotorConfigs.Slot0.kV = elevatorConstants.kV;
        leftMotorConfigs.Slot0.kG = elevatorConstants.kG;
        leftMotorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        leftMotor.getConfigurator().apply(leftMotorConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current1,
            current2,
            temp1,
            temp2,
            RPS1,
            position1
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        setpointMeters = 0;
    }

    public void setMotionMagicSetpoint(double setpointMeters) {
        this.setpointMeters = setpointMeters;
        leftMotor.setControl(motionMagicRequest.withPosition(Conversions.metersToRotations(setpointMeters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)));
    }

    public void runElevator(double voltage) {
        leftMotor.setControl(voltageOutRequest.withOutput(voltage));
    }

    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(
            current1,
            current2,
            temp1,
            temp2,
            RPS1,
            position1
        );

        inputs.setpointMeters = setpointMeters;
        inputs.elevatorVelMPS = Conversions.RPStoMPS(RPS1.getValue(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.elevatorHeightMeters = Conversions.RotationsToMeters(position1.getValue(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.currentAmps = new double[] {current1.getValue(), current2.getValue()};
        inputs.tempF = new double[] {temp1.getValue(), temp2.getValue()};
    }

    public void zeroSensor(){
        leftMotor.setPosition(0);
    }
}