package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;

import frc.robot.constants.commandConstants;

import frc.robot.commands.singles.RunIntake2;
import frc.robot.commands.singles.SetPivot;
import frc.robot.commands.singles.SetElevatorPosition;


public class ElevatorOuttakeSequence extends ParallelCommandGroup {
    public ElevatorOuttakeSequence(Elevator elevator, Intake intake, OtbIntake otbIntake) {
        addCommands(
            new SetPivot(otbIntake, commandConstants.restingDegrees),
            new SequentialCommandGroup(
                new SetElevatorPosition(elevator, commandConstants.maxHeight),
                new WaitUntilCommand(elevator::atSetpoint),
                new RunIntake2(intake, commandConstants.outakeVoltage, 0.75),
                new SetElevatorPosition(elevator, commandConstants.minHeight)
                )
        );
    }
}
