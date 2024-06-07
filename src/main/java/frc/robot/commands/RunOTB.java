package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.otb_intake.otbIntake;
import edu.wpi.first.wpilibj.RobotController;


public class RunOTB extends Command {
    private final otbIntake otbIntake;
    private final Intake intake;
    private final double voltage;
    private final double angleDegrees;
    private double stateStartTime = 0;


    public RunOTB(otbIntake otbIntake, Intake intake, double voltage, double angleDegrees) {
        this.otbIntake = otbIntake;
        this.intake = intake;
        this.voltage = voltage;
        this.angleDegrees = angleDegrees;
        addRequirements(otbIntake);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        stateStartTime = RobotController.getFPGATime() / 1.0E6; 
        otbIntake.requestIntake(angleDegrees,voltage);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        otbIntake.requestIntake(Constants.commandConstants.restingDegrees, 0.0);
    }

    @Override
    public boolean isFinished() {
        return intake.getStatorCurrent() > 40 && (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25);
    }
}
