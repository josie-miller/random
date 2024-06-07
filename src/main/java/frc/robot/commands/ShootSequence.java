package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.handoff.Handoff;
import frc.robot.subsystems.intake.Intake;

public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence(Intake intake, Handoff handoff, Shooter shooter, double time, double ratio, double intakevoltage, double handoffvoltage, double shootervelocity, double shorttime) {
        addCommands(
            new RunIntake2(intake,intakevoltage,shorttime),
            new RunHandoff(handoff, handoffvoltage, shorttime),
            new RunShooter(shooter, shootervelocity,time,ratio),
            new WaitUntilCommand(time)
        );
    }
}

//urmum