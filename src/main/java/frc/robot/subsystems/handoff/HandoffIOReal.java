package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HandoffIOReal implements HandoffIO {
    private final TalonFX handoffMotor;
    private VoltageOut handoffRequest;
    private final StatusSignal<Double> current;
    private final StatusSignal<Double> temp;
    private final StatusSignal<Double> RPS;

    public HandoffIOReal() {
        handoffMotor = new TalonFX(Constants.canIDConstants.handoffMotor, "canivore");
        handoffRequest = new VoltageOut(0).withEnableFOC(true);
        current = handoffMotor.getStatorCurrent();
        temp = handoffMotor.getDeviceTemp();
        RPS = handoffMotor.getRotorVelocity();
        
        var handoffConfigs = new TalonFXConfiguration();
        var handoffCurrentLimitConfigs = handoffConfigs.CurrentLimits;
        handoffCurrentLimitConfigs.StatorCurrentLimit = Constants.handoffConstants.statorCurrentLimit;
        handoffCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        handoffConfigs.MotorOutput.Inverted = Constants.handoffConstants.handoffInvert;

        handoffMotor.getConfigurator().apply(handoffConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS
        );

        handoffMotor.optimizeBusUtilization();
    }

    @Override
    public void setOutput(double volts) {
        handoffMotor.setControl(handoffRequest.withOutput(volts));
    }

    @Override
    public void updateInputs(HandoffIOInputs inputs) {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        inputs.current = current.getValue();
        inputs.temp = temp.getValue();
        inputs.RPS = RPS.getValue();
    }
}
