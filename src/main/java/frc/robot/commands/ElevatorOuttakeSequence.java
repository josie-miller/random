package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.Constants;

public class ElevatorOuttakeSequence extends ParallelCommandGroup {
    public ElevatorOuttakeSequence(Elevator elevator, Intake intake, OtbIntake otbIntake) {
        addCommands(
            new InstantCommand(() -> otbIntake.setPivot()),
            new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setElevatorSetpoint(Constants.commandConstants.maxHeight)),
                new WaitUntilCommand(elevator::atSetpoint),
                new RunCommand(() -> intake.runIntake(Constants.commandConstants.outakeVoltage, 0.75)),
                new InstantCommand(() -> elevator.setElevatorSetpoint(Constants.commandConstants.minHeight))
                )

        );
    }
}
