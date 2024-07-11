package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.Constants;
import frc.robot.commands.singles.RunIntake2;
import frc.robot.commands.singles.SetPivot;
import frc.robot.commands.singles.SetElevatorPosition;

public class ElevatorOuttakeSequence extends ParallelCommandGroup {
    public ElevatorOuttakeSequence(Elevator elevator, Intake intake, OtbIntake otbIntake) {
        addCommands(
            new SetPivot(otbIntake, Constants.commandConstants.restingDegrees),
            new SequentialCommandGroup(
                new SetElevatorPosition(elevator, Constants.commandConstants.maxHeight),
                new WaitUntilCommand(elevator::atSetpoint),  // Wait until the elevator reaches the setpoint
                new RunIntake2(intake, Constants.commandConstants.outakeVoltage, 0.75),
                new SetElevatorPosition(elevator, Constants.commandConstants.minHeight)
                )

        );
    }
}
