package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class OtbIntake extends SubsystemBase {
    private final OtbIntakeIO otbintakeIO;
    private OtbIntakeIOInputsAutoLogged inputs = new OtbIntakeIOInputsAutoLogged();
    private final SysIdRoutine pivotSysID;

    private double setpointVolts;
    private double pivotSetpoint;

    public OtbIntake(OtbIntakeIO otbintakeIO) {
        this.otbintakeIO = otbintakeIO;
        pivotSysID  = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> otbintakeIO.setPivotVoltage(volts.in(Volts)), null,
                    this));
        setpointVolts = 0.0;
        pivotSetpoint = 0.0;
    }

    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 110),
                this.runOnce(() -> otbintakeIO.setPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> otbintakeIO.setPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(() -> Math.abs(inputs.pivotPosDeg) > 110),
                this.runOnce(() -> otbintakeIO.setPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(() -> inputs.pivotPosDeg < 5),
                this.runOnce(() -> otbintakeIO.setPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    } 

    @Override    
    public void periodic(){
        otbintakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }

    public void requestPivotVoltage(double voltage) {
        otbintakeIO.setPivotVoltage(voltage);
    }

    public void requestSetpoint(double angleDegrees) {
        pivotSetpoint = angleDegrees;
        otbintakeIO.setPivotPosition(pivotSetpoint);
    }

    public void requestIntakeVoltage(double voltage) {
        setpointVolts = voltage;
        otbintakeIO.setIntakeVoltage(setpointVolts);
    }

     public void requestIntake(double angleDegrees, double voltage) {
        requestSetpoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }

    public double getStatorCurrent(){
        return inputs.intakeCurrent;
    }

    public double getPivotStatorCurrent(){
        return inputs.pivotCurrent;
    }
}