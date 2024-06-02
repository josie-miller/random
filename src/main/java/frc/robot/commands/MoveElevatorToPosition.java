package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToPosition extends InstantCommand {

    public MoveElevatorToPosition(Elevator elevator, double targetPosition) {
        super(() -> {
            elevator.setSetpoint(targetPosition);
            elevator.enable();
        }, elevator);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.disable();
        }
    }
}
