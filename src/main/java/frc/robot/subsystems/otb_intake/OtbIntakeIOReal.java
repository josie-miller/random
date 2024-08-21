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
    private final TalonFX intakeMotor = new TalonFX(canIDConstants.otbIntakeMotor, "rio");
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    private VoltageOut intakeMotorVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Double> intakeCurrent = intakeMotor.getStatorCurrent();
    private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp();
    private final StatusSignal<Double> intakeRPS = intakeMotor.getRotorVelocity();

   private double setpointVolts;

    public OtbIntakeIOReal() {
        
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfigs.MotorOutput.Inverted = otbIntakeConstants.intakeInvert;
        intakeConfigs.CurrentLimits.StatorCurrentLimit = otbIntakeConstants.intakeCurrentLimit;
        intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

       
        intakeMotor.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                
                intakeCurrent,
                intakeTemp,
                intakeRPS
               );

        intakeMotor.optimizeBusUtilization();

        setpointVolts = 0;
    }




    @Override
    public void setIntakeVoltage(double voltage) {
        this.setpointVolts = voltage;
        intakeMotor.setControl(intakeMotorVoltageRequest.withOutput(setpointVolts));
    }



    public void updateInputs(OtbIntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                intakeCurrent,
                intakeTemp,
                intakeRPS
        );
        inputs.intakeCurrent = intakeCurrent.getValue();
        inputs.intakeTemp = intakeTemp.getValue();
        inputs.intakeRPS = intakeRPS.getValue();
        inputs.setpointVolts = this.setpointVolts;
    }
}