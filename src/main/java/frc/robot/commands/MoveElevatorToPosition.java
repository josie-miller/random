package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToPosition extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    public MoveElevatorToPosition(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(targetPosition);
        elevator.enable();
    }

    @Override
    public void execute() {
        // Additional logic if needed
    }

    @Override
    public void end(boolean interrupted) {
        elevator.disable();
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }
}
