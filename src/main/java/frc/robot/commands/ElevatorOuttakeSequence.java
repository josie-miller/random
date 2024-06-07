package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;

public class ElevatorOuttakeSequence extends SequentialCommandGroup {
    public ElevatorOuttakeSequence(Elevator elevator, Intake intake) {
        addCommands(
            new MoveElevatorToPosition(elevator, Constants.commandConstants.maxHeight),
            new WaitUntilCommand(elevator::atSetpoint),  // Wait until the elevator reaches the setpoint
            new RunOutake(intake, Constants.commandConstants.outakeVoltage, 0.75),
            new MoveElevatorToPosition(elevator, Constants.commandConstants.minHeight)
        );
    }
}
