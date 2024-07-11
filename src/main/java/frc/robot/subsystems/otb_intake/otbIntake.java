package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants;

public class OtbIntake extends SubsystemBase {
    private final OtbIntakeIO otbintakeIO;
    private OtbIntakeIOInputsAutoLogged inputs = new OtbIntakeIOInputsAutoLogged();

    private double setpointVolts;
    private double pivotSetpoint;

    public OtbIntake(OtbIntakeIO otbintakeIO) {
        this.otbintakeIO = otbintakeIO;
        setpointVolts = 0.0;
        pivotSetpoint = 0.0;
    }
    
    public void periodic(){
        otbintakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }

    public void requestPivotVoltage(double voltage) {
        otbintakeIO.setPivotVoltage(voltage);
    }

    public void requestSetpoint(double angleDegrees) {
        pivotSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio);
        otbintakeIO.setPivotPosition(pivotSetpointRotations);
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