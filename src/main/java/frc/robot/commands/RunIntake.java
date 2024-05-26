package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class RunIntake extends Command {
    private final Intake intake;
    private final double voltage;

    public RunIntake(Intake intake, double voltage) {
        this.intake = intake;
        this.voltage = voltage;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
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
        return false; 
    }
}
