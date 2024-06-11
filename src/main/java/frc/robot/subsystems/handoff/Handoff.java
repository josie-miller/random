package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Handoff extends SubsystemBase{
    private final TalonFX handoffMotor;
    private VoltageOut handoffRequest;
    private final StatusSignal<Double> current;
    private final StatusSignal<Double> temp;
    private final StatusSignal<Double> RPS;
    private double setpointVolts;

    public Handoff(){
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

        setpointVolts = 0.0;
    }

    public void runHandoff(double voltage){
        setpointVolts = voltage;
        handoffMotor.setControl(handoffRequest.withOutput(voltage));
    }

    public Double getStatorCurrent(){
        return current.getValue();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        SmartDashboard.putNumber("Handoff Voltage", setpointVolts);
        SmartDashboard.putNumber("Handoff Current", current.getValue());
        SmartDashboard.putNumber("Handoff Temperature", temp.getValue());
        SmartDashboard.putNumber("Handoff Speed (RPS)", RPS.getValue());
    }
}
