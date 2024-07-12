package frc.robot.subsystems.otb_intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.commons.Conversions;
import frc.robot.constants.canIDConstants;
import frc.robot.constants.otbIntakeConstants;

public class OtbIntakeIOReal implements OtbIntakeIO {
    private final TalonFX pivotMotor = new TalonFX(canIDConstants.otbIntakePivotMotor, "canivore");
    private final TalonFX intakeMotor = new TalonFX(canIDConstants.otbIntakeMotor, "rio");
    private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    private MotionMagicVoltage pivotMotorMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private VoltageOut pivotMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private VoltageOut intakeMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Double> pivotCurrent = pivotMotor.getStatorCurrent();
    private final StatusSignal<Double> pivotTemp = pivotMotor.getDeviceTemp();
    private final StatusSignal<Double> pivotRPS = pivotMotor.getRotorVelocity();
    private final StatusSignal<Double> pivotPos = pivotMotor.getRotorPosition();
    private final StatusSignal<Double> intakeCurrent = intakeMotor.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intakeMotor.getRotorVelocity();

   private double pivotSetpoint;
   private double setpointVolts;

    public OtbIntakeIOReal() {
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.MotorOutput.Inverted = otbIntakeConstants.pivotInvert;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = otbIntakeConstants.pivotCurrentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfigs.MotorOutput.Inverted = otbIntakeConstants.intakeInvert;
        intakeConfigs.CurrentLimits.StatorCurrentLimit = otbIntakeConstants.intakeCurrentLimit;
        intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotConfigs.Slot0.kP = otbIntakeConstants.kP; 
        pivotConfigs.Slot0.kI = otbIntakeConstants.kI;
        pivotConfigs.Slot0.kD = otbIntakeConstants.kD;
        pivotConfigs.Slot0.kS = otbIntakeConstants.kS; 
        pivotConfigs.Slot0.kV = otbIntakeConstants.kV; 
        pivotConfigs.Slot0.kA = otbIntakeConstants.kA;
        pivotConfigs.Slot0.kG = otbIntakeConstants.kG;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = otbIntakeConstants.CruiseVelocity;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = otbIntakeConstants.Acceleration;
        pivotConfigs.MotionMagic.MotionMagicJerk = otbIntakeConstants.Jerk;

        pivotMotor.setPosition(0);

        pivotMotor.getConfigurator().apply(pivotConfigs);
        intakeMotor.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp,
                intakeCurrent,
                intakeTemp,
                intakeRPS
               );

        intakeMotor.optimizeBusUtilization();
        pivotMotor.optimizeBusUtilization();

        pivotSetpoint = 0;
        setpointVolts = 0;
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(pivotMotorVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setPivotPosition(double angleDegrees) {
        this.pivotSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, otbIntakeConstants.gearRatio);
        pivotMotor.setControl(pivotMotorMotionMagicRequest.withPosition(pivotSetpointRotations));
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        this.setpointVolts = voltage;
        intakeMotor.setControl(intakeMotorVoltageRequest.withOutput(setpointVolts));
    }

    @Override
    public void zeroPosition() {
        pivotMotor.setPosition(0);
    }

    public void updateInputs(OtbIntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp,
                intakeCurrent,
                intakeTemp,
                intakeRPS
        );
        inputs.intakeCurrent = intakeCurrent.getValue();
        inputs.intakeTemp = intakeTemp.getValue();
        inputs.intakeRPS = intakeRPS.getValue();
        inputs.pivotCurrent = pivotCurrent.getValue();
        inputs.pivotTemp = pivotTemp.getValue();
        inputs.pivotRPS = pivotRPS.getValue();
        inputs.setpointVolts = this.setpointVolts;
        inputs.pivotSetpointDeg = this.pivotSetpoint;
    }
}
