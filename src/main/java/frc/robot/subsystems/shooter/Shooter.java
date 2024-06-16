package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;

    public Shooter(ShooterIO shooterIO) {
        this.shooterIO = shooterIO;
    }

    public void setVelocity(double velocity, double ratio) {
        shooterIO.setVelocity(velocity, ratio);
    }

    public void setVoltage(double voltage) {
        shooterIO.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
        shooterIO.updateInputs(inputs);
        SmartDashboard.putNumber("Shooter Current", inputs.current);
        SmartDashboard.putNumber("Shooter Temperature", inputs.temp);
        SmartDashboard.putNumber("Shooter Speed (RPS)", inputs.RPS);
    }
}
