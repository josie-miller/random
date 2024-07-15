package frc.robot.autons.modes;

import com.choreo.lib.Choreo;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.OtbIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.sequences.IntakeSequence;
import frc.robot.commands.sequences.ShootSequence;

import frc.robot.constants.commandConstants;

public class TwoPieceMid extends SequentialCommandGroup {

    public TwoPieceMid(Swerve swerve, Intake intake, OtbIntake otbIntake, Handoff handoff, Shooter shooter){
        addRequirements(swerve, intake, otbIntake, handoff, shooter);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new ShootSequence(intake, handoff, shooter, commandConstants.midShootTime, commandConstants.midShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootMidVelocity, commandConstants.midShortTime),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                swerve.runChoreoTraj(Choreo.getTrajectory("midCloseForwardMID"), true),
                new IntakeSequence(otbIntake, intake, handoff)
            ),
            new WaitCommand(1),
            swerve.runChoreoTraj(Choreo.getTrajectory("midCloseBackwardMID")),
            new ShootSequence(intake, handoff, shooter, commandConstants.midShootTime, commandConstants.midShootRatio, commandConstants.handoffIntakeVoltage, commandConstants.handoffShooterVoltage, commandConstants.shootMidVelocity, commandConstants.midShortTime)
        );
    }

}
