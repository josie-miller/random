package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.commons.Conversions;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.constants.elevatorConstants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine elevatorRoutine;
    private double setpointMeters;

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
        elevatorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(2),null, 
                    (state) -> SignalLogger.writeString("state", state.toString())), 
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> elevatorIO.setVoltage(volts.in(Volts)), null, 
                    this));
    }

    public void setSetpoint(double setpointMeters) {
        this.setpointMeters = setpointMeters;
        double setpointRotations = Conversions.metersToRotations(setpointMeters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        elevatorIO.setMotionMagicSetpoint(setpointRotations);
    }

    public Command elevatorSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            elevatorRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2),
                this.runOnce(() -> elevatorIO.setVoltage(0)),
                Commands.waitSeconds(1),
            elevatorRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.elevatorHeightMeters < 0.2),
                this.runOnce(() -> elevatorIO.setVoltage(0)),
                Commands.waitSeconds(1),  
            elevatorRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2),
                this.runOnce(() -> elevatorIO.setVoltage(0)),
                Commands.waitSeconds(1),  
            elevatorRoutine
                .dynamic(Direction.kReverse)
                .until(() -> inputs.elevatorHeightMeters < 0.2),
                this.runOnce(() -> elevatorIO.setVoltage(0)),
                Commands.waitSeconds(1), 
            this.runOnce(() -> SignalLogger.stop())
        );
    }

    @Override
    public void periodic() {
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
        elevatorIO.setVoltage(0);
    }
    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        elevatorIO.updateInputs(inputs);
    }
}