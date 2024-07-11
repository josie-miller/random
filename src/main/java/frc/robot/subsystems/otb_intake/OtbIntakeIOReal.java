package frc.robot.subsystems.otb_intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;
import frc.robot.Conversions;

public class OtbIntakeIOReal implements OtbIntakeIO {
    private final TalonFX pivotMotor = new TalonFX(Constants.canIDConstants.otbIntakePivotMotor, "canivore");
    private final TalonFX intakeMotor = new TalonFX(Constants.canIDConstants.otbIntakeMotor, "rio");
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
        var pivotMotorOuputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOuputConfigs.Inverted = Constants.otbIntakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = Constants.otbIntakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = Constants.otbIntakeConstants.intakeInvert;

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = Constants.otbIntakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

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
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(pivotMotorVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setPivotPosition(double angleDegrees) {
        pivotSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio);
        pivotMotor.setControl(pivotMotorMotionMagicRequest.withPosition(pivotSetpointRotations));
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        setpointVolts = voltage;
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
        inputs.pivotSetpointDeg = pivotSetpoint;
    }
}