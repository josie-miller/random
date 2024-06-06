package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class ElevatorOuttakeSequence extends SequentialCommandGroup {
    public ElevatorOuttakeSequence(Elevator elevator, Intake intake) {
        addCommands(
            new MoveElevatorToPosition(elevator, 0.45),
            new WaitUntilCommand(elevator::atSetpoint),  // Wait until the elevator reaches the setpoint
            new RunOutake(intake, -3.5, 0.75),
            new MoveElevatorToPosition(elevator, 0)
        );
    }
}
