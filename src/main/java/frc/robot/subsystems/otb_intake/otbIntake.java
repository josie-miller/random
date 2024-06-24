package frc.robot.subsystems.otb_intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants;

public class OtbIntake extends SubsystemBase {
    private final OtbIntakeIO intakeIO;
    private OtbIntakeIOInputsAutoLogged inputs = new OtbIntakeIOInputsAutoLogged();

    public OtbIntake(OtbIntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }
    
    public void Loop(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
    }

    public void requestPivotVoltage(double voltage) {
        intakeIO.setPivotVoltage(voltage);
    }
/* 
    public void requestSetpoint(double angleDegrees) {
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio);
        intakeIO.setPivotPosition(pivotSetpointRotations);
    }*/

    public Command setPivot(double angleDegrees) {
        return this.runOnce(() -> intakeIO.setPivotPosition(Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio)));
    }

    public Command setPivot() {
        return this.runOnce(() -> intakeIO.setPivotPosition(Conversions.DegreesToRotations(Constants.commandConstants.restingDegrees, Constants.otbIntakeConstants.gearRatio)));
    }

    public Command requestOTB(double angleDegrees, double voltage) {
        return setPivot(angleDegrees)
        .alongWith(this.run(() -> intakeIO.setIntakeVoltage(voltage)));
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
        return this.runOnce(() -> intakeIO.setPivotPosition(Conversions.DegreesToRotations(0, Constants.otbIntakeConstants.gearRatio)));
    }

    public double getStatorCurrent(){
        return inputs.intakeCurrent;
    }

    public double getPivotStatorCurrent(){
        return inputs.pivotCurrent;
    }
}
