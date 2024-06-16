package frc.robot.subsystems.otb_intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants;

public class OtbIntake extends SubsystemBase {
    private final OtbIntakeIO intakeIO;
    private double pivotMotorSetpoint;

    public OtbIntake(OtbIntakeIO intakeIO) {
        this.intakeIO = intakeIO;
    }

    public void requestPivotVoltage(double voltage) {
        intakeIO.setPivotVoltage(voltage);
    }

    public void requestSetpoint(double angleDegrees) {
        pivotMotorSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, Constants.otbIntakeConstants.gearRatio);
        intakeIO.setPivotPosition(pivotSetpointRotations);
    }

    public void requestIntakeVoltage(double voltage) {
        intakeIO.setIntakeVoltage(voltage);
    }

    public void requestIntake(double angleDegrees, double voltage) {
        requestSetpoint(angleDegrees);
        requestIntakeVoltage(voltage);
    }

    public void zeroPosition(){
        intakeIO.zeroPosition();
    }

    @Override
    public void periodic() {
        OtbIntakeIO.OtbIntakeIOInputs inputs = new OtbIntakeIO.OtbIntakeIOInputs();
        intakeIO.updateInputs(inputs);
        SmartDashboard.putNumber("Intake Current", inputs.intakeCurrent);
        SmartDashboard.putNumber("Intake Temperature", inputs.intakeTemp);
        SmartDashboard.putNumber("Intake Speed (RPS)", inputs.intakeRPS);
        SmartDashboard.putNumber("Pivot Current", inputs.pivotCurrent);
        SmartDashboard.putNumber("Pivot Temperature", inputs.pivotTemp);
        SmartDashboard.putNumber("Pivot Speed (RPS)", inputs.pivotRPS);
    }
}
