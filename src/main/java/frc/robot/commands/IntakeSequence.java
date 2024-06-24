package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;

public class IntakeSequence extends ParallelDeadlineGroup {
    public IntakeSequence(OtbIntake otbIntake, Intake intake, Handoff handoff) {
        super(
            new RunCommand(() -> otbIntake.requestOTB(Constants.commandConstants.floorDegrees, Constants.commandConstants.otbVoltage))
        );
        setDeadline(new RunCommand(() -> intake.runIntake(Constants.commandConstants.intakeVoltage)));
    }
}
