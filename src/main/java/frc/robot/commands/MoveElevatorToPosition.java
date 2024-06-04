package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToPosition extends InstantCommand {
    private final Elevator elevator;
    private final double targetPosition;

    public MoveElevatorToPosition(Elevator elevator, double targetPosition) {
        super();
        this.elevator = elevator;
        this.targetPosition = -targetPosition; // idk why but it is what it is
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.disable();
        }
    }
}
