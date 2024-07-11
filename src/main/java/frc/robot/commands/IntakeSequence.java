package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants;

public class IntakeSequence extends ParallelDeadlineGroup {
    public IntakeSequence(OtbIntake otbIntake, Intake intake, Handoff handoff) {
        super(
           // new RunIntake(intake, Constants.commandConstants.intakeVoltage),
            new RunOTB(otbIntake, Constants.commandConstants.otbVoltage, Constants.commandConstants.floorDegrees)

            
        );
        setDeadline(new RunIntake(intake, Constants.commandConstants.intakeVoltage));
    }
}
