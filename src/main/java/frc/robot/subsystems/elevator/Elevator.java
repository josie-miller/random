package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private double setpointMeters;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void setSetpoint(double setpointMeters) {
        this.setpointMeters = setpointMeters;
        double setpointRotations = Conversions.metersToRotations(setpointMeters, Constants.elevatorConstants.wheelCircumferenceMeters, Constants.elevatorConstants.gearRatio);
        elevatorIO.setMotionMagicSetpoint(setpointRotations);
    }

    @Override
    public void periodic() {
        ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
        elevatorIO.updateInputs(inputs);
        SmartDashboard.putNumber("Elevator Position", inputs.position);
        SmartDashboard.putNumber("Elevator Current", inputs.current);
        SmartDashboard.putNumber("Elevator Temperature", inputs.temperature);
        SmartDashboard.putNumber("Elevator Speed (RPS)", inputs.RPS);
    }

    public void zeroSensor() {
        elevatorIO.zeroSensorPosition();
    }

    public boolean atSetpoint() {
        double currentPosMeters = Conversions.rotationsToMeters(elevatorIO.getPosition(), Constants.elevatorConstants.wheelCircumferenceMeters, Constants.elevatorConstants.gearRatio);
        return Math.abs(currentPosMeters - setpointMeters) < Constants.elevatorConstants.ToleranceMeters;
    }

    public void disable() {
        elevatorIO.disableMotors();
    }
}
