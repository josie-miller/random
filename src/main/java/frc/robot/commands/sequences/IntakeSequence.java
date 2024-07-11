package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;

import frc.robot.constants.commandConstants;

import frc.robot.commands.singles.RunIntake;
import frc.robot.commands.singles.RunOTB;

public class IntakeSequence extends ParallelDeadlineGroup {
    public IntakeSequence(OtbIntake otbIntake, Intake intake, Handoff handoff) {
        super(
            new RunOTB(otbIntake, commandConstants.otbVoltage, commandConstants.floorDegrees)
        );
        setDeadline(new RunIntake(intake, commandConstants.intakeVoltage));
    }
}
