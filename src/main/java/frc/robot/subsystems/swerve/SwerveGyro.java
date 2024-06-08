package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class SwerveGyro {
    Pigeon2 pigeon;
    private final StatusSignal<Double> positionDegRaw;
    private final StatusSignal<Double> pitchDeg;
    private final StatusSignal<Double> rollDeg;

    public SwerveGyro(int pigeonID){
        pigeon = new Pigeon2(pigeonID, "canivore");
        positionDegRaw = pigeon.getYaw();
        pitchDeg = pigeon.getPitch();
        rollDeg = pigeon.getRoll();


        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            positionDegRaw,
            pitchDeg,
            rollDeg
        );

        pigeon.optimizeBusUtilization();
    }

    public void reset() {
        pigeon.setYaw(0.0);
    }

    public void setPosition(double yawDegrees){
        pigeon.setYaw(yawDegrees);
    }

}