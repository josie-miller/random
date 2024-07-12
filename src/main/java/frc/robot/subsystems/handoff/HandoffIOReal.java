package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.constants.canIDConstants;
import frc.robot.constants.handoffConstants;

public class HandoffIOReal implements HandoffIO {
    private final TalonFX handoff = new TalonFX(canIDConstants.handoffMotor, "canivore");
    private final TalonFXConfiguration handoffConfigs = new TalonFXConfiguration();

    private VoltageOut handoffRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Double> current = handoff.getStatorCurrent();
    private final StatusSignal<Double> temp = handoff.getDeviceTemp();
    private final StatusSignal<Double> RPS = handoff.getRotorVelocity();

    private double setpointVolts;

    public HandoffIOReal() {
        handoffConfigs.CurrentLimits.StatorCurrentLimit = handoffConstants.statorCurrentLimit;
        handoffConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        handoffConfigs.MotorOutput.Inverted = handoffConstants.handoffInvert;
        
        handoff.getConfigurator().apply(handoffConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS);

        handoff.optimizeBusUtilization();

        setpointVolts = 0;
    }

    public void updateInputs(HandoffIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        inputs.appliedVolts = handoffRequest.Output;
        inputs.setpointVolts = this.setpointVolts;
        inputs.currentAmps = current.getValue();
        inputs.tempF = temp.getValue();
        inputs.handoffRPS = RPS.getValue();
    }

    public void runHandoff(double output) {
        this.setpointVolts = output;
        handoff.setControl(handoffRequest.withOutput(output));
    }
}
