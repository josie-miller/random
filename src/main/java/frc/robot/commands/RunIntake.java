package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.handoff.Handoff;

public class RunIntake extends Command {
    private final Intake intake;
    private final double voltage;
    private final Handoff handoff;
    private double stateStartTime = 0;


    public RunIntake(Intake intake, Handoff handoff, double voltage) {
        this.intake = intake;
        this.handoff = handoff;
        this.voltage = voltage;
        addRequirements(intake);
        addRequirements(handoff);    }

    @Override
    public void initialize() {
        stateStartTime = RobotController.getFPGATime() / 1.0E6; 
        intake.runIntake(voltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntake(0.0);
    }

    @Override
    public boolean isFinished() {
        return handoff.getStatorCurrent() > 5 && (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25);
    }
}
