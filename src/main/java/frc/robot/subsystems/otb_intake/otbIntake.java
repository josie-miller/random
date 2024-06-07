package frc.robot.subsystems.otb_intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

public class otbIntake extends SubsystemBase {
    private final TalonFX pivotMotor = new TalonFX(Constants.canIDConstants.otbIntakePivotMotor, "canivore");
    private final TalonFX intakeMotor = new TalonFX(Constants.canIDConstants.otbIntakeMotor, "rio");
    private final TalonFXConfigurator pivotConfigurator;
    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfigurator intakeConfigurator;
    private final TalonFXConfiguration intakeConfigs;
    MotionMagicVoltage pivotMotorMotionMagicRequest;
    VoltageOut pivotMotorVoltageRequest;
    VoltageOut intakeMotorVoltageRequest;
    double pivotMotorSetpoint;

    private final StatusSignal<Double> pivotCurrent = pivotMotor.getStatorCurrent();
    private final StatusSignal<Double> pivotTemp = pivotMotor.getDeviceTemp();
    private final StatusSignal<Double> pivotRPS = pivotMotor.getRotorVelocity();
    private final StatusSignal<Double> pivotPos = pivotMotor.getRotorPosition();

    private final StatusSignal<Double> intakeCurrent = intakeMotor.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intakeMotor.getRotorVelocity();


    public otbIntake() {
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

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp
                
               );

        intakeMotor.optimizeBusUtilization();

    }

    public void requestPivotVoltage(double voltage) {
        pivotMotor.setControl(pivotMotorVoltageRequest.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees) {
        pivotMotorSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio);
        pivotMotor.setControl(pivotMotorMotionMagicRequest.withPosition(pivotSetpointRotations));
    }

    public void requestIntakeVoltage(double voltage) {
        intakeMotor.setControl(intakeMotorVoltageRequest.withOutput(voltage));
    }


    public void requestIntake(double angleDegrees, double voltage) {
        requestSetpoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }

    public void zeroPosition(){
        pivotMotor.setPosition(0);
    }

    @Override
    public void periodic() {
    BaseStatusSignal.refreshAll(intakeCurrent, intakeTemp, intakeRPS);
    SmartDashboard.putNumber("Intake Current", intakeCurrent.getValue());
    SmartDashboard.putNumber("Intake Temperature", intakeTemp.getValue());
    SmartDashboard.putNumber("Intake Speed (RPS)", intakeRPS.getValue());
    SmartDashboard.putNumber("Pivot Current", pivotCurrent.getValue());
    SmartDashboard.putNumber("Pivot Temperature", pivotTemp.getValue());
    SmartDashboard.putNumber("Pivot Speed (RPS)", pivotRPS.getValue());
    }
    
}
