package frc.robot.commands.singles;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj.RobotController;

public class RunIntake2 extends Command {
    private final Intake intake;
    private final double voltage;
    private final double time;
    private double stateStartTime = 0;


    public RunIntake2(Intake intake, double voltage, double time) {
        this.intake = intake;
        this.voltage = voltage;
        this.time = time;
        addRequirements(intake);
    }

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
        return (RobotController.getFPGATime() / 1.0E6 - stateStartTime > time);
    }
}
