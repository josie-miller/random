package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor;
    private VoltageOut intakeRequest;
    private final StatusSignal<Double> current;
    private final StatusSignal<Double> temp;
    private final StatusSignal<Double> RPS;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(Constants.canIDConstants.intakeMotor, "canivore");
        intakeRequest = new VoltageOut(0).withEnableFOC(true);
        current = intakeMotor.getStatorCurrent();
        temp = intakeMotor.getDeviceTemp();
        RPS = intakeMotor.getRotorVelocity();

        var intakeConfigs = new TalonFXConfiguration();
        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = Constants.intakeConstants.statorCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        intakeConfigs.MotorOutput.Inverted = Constants.intakeConstants.intakeInvert;

        intakeMotor.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS
        );

        intakeMotor.optimizeBusUtilization();
    }

    @Override
    public void setOutput(double volts) {
        intakeMotor.setControl(intakeRequest.withOutput(volts));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        inputs.appliedVolts = intakeRequest.Output;
        inputs.current = current.getValue();
        inputs.tempF = temp.getValue();
        inputs.intakeRPS = RPS.getValue();
    }
    @Override
    public Double getStatorCurrent() {
        return current.getValue();
    }
}
