package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Conversions;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double setpointMeters;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void setSetpoint(double setpointMeters) {
        this.setpointMeters = setpointMeters;
        double setpointRotations = Conversions.metersToRotations(setpointMeters, Constants.elevatorConstants.wheelCircumferenceMeters, Constants.elevatorConstants.gearRatio);
        elevatorIO.setMotionMagicSetpoint(setpointRotations);
    }

    public void Loop() {
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
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
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorIO.updateInputs(inputs);
    }
}
