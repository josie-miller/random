package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private VoltageOut intakeRequest;
    private final StatusSignal<Double> current;
    private final StatusSignal<Double> temp;
    private final StatusSignal<Double> RPS;
    private double setpointVolts;

    public Intake() {
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

        setpointVolts = 0;
    }

    public void runIntake(double voltage) {
        setpointVolts = voltage;
        intakeMotor.setControl(intakeRequest.withOutput(voltage));
    }

    
    public Double getStatorCurrent(){
        return current.getValue();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        SmartDashboard.putNumber("Intake Voltage", setpointVolts);
        SmartDashboard.putNumber("Intake Current", current.getValue());
        SmartDashboard.putNumber("Intake Temperature", temp.getValue());
        SmartDashboard.putNumber("Intake Speed (RPS)", RPS.getValue());
    }
}