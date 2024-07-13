package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.constants.canIDConstants;
import frc.robot.constants.intakeConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intake = new TalonFX(canIDConstants.intakeMotor, "canivore");
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    private VoltageOut intakeRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Double> current= intake.getStatorCurrent();
    private final StatusSignal<Double> temp = intake.getDeviceTemp();
    private final StatusSignal<Double> RPS = intake.getRotorVelocity();

    private double setpointVolts;

    public IntakeIOReal() {
        intakeConfigs.CurrentLimits.StatorCurrentLimit = intakeConstants.statorCurrentLimit;
        intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfigs.MotorOutput.Inverted = intakeConstants.intakeInvert;
        
        intake.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temp,
            RPS);

        intake.optimizeBusUtilization();

        setpointVolts = 0;
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            current,
            temp,
            RPS
        );
        inputs.appliedVolts = intakeRequest.Output;
        inputs.setpointVolts = this.setpointVolts;
        inputs.currentAmps = current.getValue();
        inputs.tempF = temp.getValue();
        inputs.intakeRPS = RPS.getValue();
    }

    public void runIntake(double voltage) {
        this.setpointVolts = voltage;
        intake.setControl(intakeRequest.withOutput(voltage));
    }

}