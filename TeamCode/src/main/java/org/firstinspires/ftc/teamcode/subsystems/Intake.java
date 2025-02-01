package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake extends SubsystemBase {
    public static double RUNNING_SPEED = 0.5;

    private final Motor intake;
    private boolean running = false;
    private boolean isForward = true;

    public Intake(final HardwareMap hwMap) {
        intake = new Motor(hwMap,"intake", Motor.GoBILDA.RPM_312);
        update();
    }

    public boolean getRunning() {
        return running;
    }

    public void setRunning(boolean running) {
        this.running = running;
        update();
    }

    public void toggle() {
        running = !running;
        update();
    }

    public boolean getForward() {
        return isForward;
    }

    public void setForward(boolean isForward) {
        this.isForward = isForward;
        update();
    }

    public void toggleForward() {
        isForward = !isForward;
        update();
    }

    public void update() {
        intake.setInverted(isForward);
        intake.set(running ? RUNNING_SPEED : 0);
    }
}