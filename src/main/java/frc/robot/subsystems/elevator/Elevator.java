package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.Conversions;
import frc.robot.constants.elevatorConstants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double setpointMeters;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void setSetpoint(double setpointMeters) {
        this.setpointMeters = setpointMeters;
        double setpointRotations = Conversions.metersToRotations(setpointMeters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        elevatorIO.setMotionMagicSetpoint(setpointRotations);
    }

    public void Loop() {
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void zeroSensor() {
        elevatorIO.zeroSensor();
    }

    public boolean atSetpoint() {
        return Math.abs(inputs.elevatorHeightMeters - setpointMeters) < elevatorConstants.ToleranceMeters;
    }

    public double getPosition() {
        return inputs.elevatorHeightMeters;
    }

    public void disable() {
        elevatorIO.runElevator(0);
    }
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorIO.updateInputs(inputs);
    }
}