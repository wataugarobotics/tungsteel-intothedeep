package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Basket extends SubsystemBase {
    private final SimpleServo servo;

    private boolean isOpen = false;

    public static double MIN_ANGLE = 0.0;
    public static double MAX_ANGLE = 100.0;

    public Basket(HardwareMap hwMap) {
        servo = new SimpleServo(hwMap, "basket", MIN_ANGLE, MAX_ANGLE);
        update();
    }

    public void open() {
        isOpen = true;
        update();
    }

    public void close() {
        isOpen = false;
        update();
    }

    public void toggle() {
        isOpen = !isOpen;
        update();
    }

    public void update() {
        servo.turnToAngle(isOpen ? MAX_ANGLE : MIN_ANGLE);
    }
}