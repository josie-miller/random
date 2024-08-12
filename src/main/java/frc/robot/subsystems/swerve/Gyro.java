package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {
    private final Pigeon2 pigeon;

    public Gyro() {
        pigeon = new Pigeon2(Constants.canIDConstants.pigeon);
        Pigeon2Configuration config = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(config);
        pigeon.setYaw(0);
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
    }

    public void reset() {
        pigeon.setYaw(0);
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }
}
