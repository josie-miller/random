package frc.robot.commands.singles;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;

public class SetElevatorPosition extends InstantCommand {
    private final Elevator elevator;
    private final double targetPosition;

    public SetElevatorPosition(Elevator elevator, double targetPosition) {
        super();
        this.elevator = elevator;
        this.targetPosition = targetPosition;
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
