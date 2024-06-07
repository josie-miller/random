package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.otb_intake.otbIntake;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;

public class IntakeSequence extends SequentialCommandGroup {
    public IntakeSequence(otbIntake otbIntake, Intake intake, Handoff handoff) {
        addCommands(
            new RunOTB(otbIntake, intake, Constants.commandConstants.otbVoltage, Constants.commandConstants.floorDegrees),
            new RunIntake(intake, handoff, Constants.commandConstants.intakeVoltage)
        );
    }
}
