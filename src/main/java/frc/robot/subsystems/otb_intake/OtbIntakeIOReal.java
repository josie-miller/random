package frc.robot.subsystems.otb_intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import frc.robot.Constants;

public class OtbIntakeIOReal implements OtbIntakeIO {
    private final TalonFX pivotMotor;
    private final TalonFX intakeMotor;
    private final TalonFXConfigurator pivotConfigurator;
    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfigurator intakeConfigurator;
    private final TalonFXConfiguration intakeConfigs;
    private MotionMagicVoltage pivotMotorMotionMagicRequest;
    private VoltageOut pivotMotorVoltageRequest;
    private VoltageOut intakeMotorVoltageRequest;

    private final StatusSignal<Double> pivotCurrent;
    private final StatusSignal<Double> pivotTemp;
    private final StatusSignal<Double> pivotRPS;
    private final StatusSignal<Double> pivotPos;

    private final StatusSignal<Double> intakeCurrent;
    private final StatusSignal<Double> intakeTemp;
    private final StatusSignal<Double> intakeRPS;

    public OtbIntakeIOReal() {
        pivotMotor = new TalonFX(Constants.canIDConstants.otbIntakePivotMotor, "canivore");
        intakeMotor = new TalonFX(Constants.canIDConstants.otbIntakeMotor, "rio");
        pivotConfigurator = pivotMotor.getConfigurator();
        pivotConfigs = new TalonFXConfiguration();
        intakeConfigurator = intakeMotor.getConfigurator();
        intakeConfigs = new TalonFXConfiguration();

        var pivotMotorOuputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOuputConfigs.Inverted = Constants.otbIntakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = Constants.otbIntakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kP = Constants.otbIntakeConstants.kP; 
        slot0Configs.kI = Constants.otbIntakeConstants.kI;
        slot0Configs.kD = Constants.otbIntakeConstants.kD;
        slot0Configs.kS = Constants.otbIntakeConstants.kS; 
        slot0Configs.kV = Constants.otbIntakeConstants.kV; 
        slot0Configs.kA = Constants.otbIntakeConstants.kA;
        slot0Configs.kG = Constants.otbIntakeConstants.kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.otbIntakeConstants.CruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = Constants.otbIntakeConstants.Acceleration;
        motionMagicConfigs.MotionMagicJerk = Constants.otbIntakeConstants.Jerk;

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = Constants.otbIntakeConstants.intakeInvert;

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = Constants.otbIntakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivotMotorMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        pivotMotor.setPosition(0);

        pivotConfigurator.apply(pivotConfigs);
        intakeConfigurator.apply(intakeConfigs);

        pivotCurrent = pivotMotor.getStatorCurrent();
        pivotTemp = pivotMotor.getDeviceTemp();
        pivotRPS = pivotMotor.getRotorVelocity();
        pivotPos = pivotMotor.getRotorPosition();

        intakeCurrent = intakeMotor.getStatorCurrent();
        intakeTemp = intakeMotor.getDeviceTemp();
        intakeRPS = intakeMotor.getRotorVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp
               );

        intakeMotor.optimizeBusUtilization();
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(pivotMotorVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setPivotPosition(double position) {
        pivotMotor.setControl(pivotMotorMotionMagicRequest.withPosition(position));
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setControl(intakeMotorVoltageRequest.withOutput(voltage));
    }

    @Override
    public void zeroPosition() {
        pivotMotor.setPosition(0);
    }

    @Override
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
    }
}
