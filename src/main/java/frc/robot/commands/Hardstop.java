package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.otbIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants;

public class Hardstop extends ParallelCommandGroup {
    public Hardstop(Elevator elevator, Intake intake, otbIntake otbIntake, Shooter shooter, Handoff handoff) {
        addCommands(
            new SequentialCommandGroup(
                new RunIntake(intake,0),
                new InstantCommand(() -> elevator.disable()),
                new RunCommand(() -> shooter.setVoltage(0)),
                new RunCommand(() -> otbIntake.requestIntakeVoltage(0)),
                new RunCommand(() -> handoff.runHandoff(0))
                )

        );
    }
}
