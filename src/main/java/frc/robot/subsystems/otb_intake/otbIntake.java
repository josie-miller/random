package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants;

public class OtbIntake extends SubsystemBase {
    private final OtbIntakeIO otbintakeIO;
    private OtbIntakeIOInputsAutoLogged inputs = new OtbIntakeIOInputsAutoLogged();

    public OtbIntake(OtbIntakeIO otbintakeIO) {
        this.otbintakeIO = otbintakeIO;
    }
    
    public void periodic(){
        otbintakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }

    public void requestPivotVoltage(double voltage) {
        otbintakeIO.setPivotVoltage(voltage);
    }
/* 
    public void requestSetpoint(double angleDegrees) {
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio);
        intakeIO.setPivotPosition(pivotSetpointRotations);
    }*/

    public Command setPivot(double angleDegrees) {
        return this.runOnce(() -> otbintakeIO.setPivotPosition(Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio)));
    }

    public Command setPivot() {
        return this.runOnce(() -> otbintakeIO.setPivotPosition(Conversions.DegreesToRotations(Constants.commandConstants.restingDegrees, Constants.otbIntakeConstants.gearRatio)));
    }

    public Command requestOTB(double voltage) {
        return this.run(() -> otbintakeIO.setIntakeVoltage(voltage));
    }
/*
    public void requestIntakeVoltage(double voltage) {
        intakeIO.setIntakeVoltage(voltage);
    }

     public void requestIntake(double angleDegrees, double voltage) {
        requestSetpoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }*/

    public Command zeroOTB(){
        return this.runOnce(() -> otbintakeIO.setPivotPosition(Conversions.DegreesToRotations(0, Constants.otbIntakeConstants.gearRatio)));
    }

    public double getStatorCurrent(){
        return inputs.intakeCurrent;
    }

    public double getPivotStatorCurrent(){
        return inputs.pivotCurrent;
    }
}
