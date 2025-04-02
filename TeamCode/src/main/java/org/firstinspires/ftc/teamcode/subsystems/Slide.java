package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slide extends SubsystemBase {
    public static double KP = 0.04;
    public static double KI = 0.0001;
    public static double KD = 0.0008;
    public static double KF = 0.0001;
    public static double TE = 50.0;
    public static double TD = 5.0;

    public static double PAST_BASKET_POS = mmToTicks(204);

    public static double speedMod = 0.75;

    private static final double TICKS_PER_MM =  384.5 / 112;
    private final double MAX_LENGTH = mmToTicks(960);

    private final MotorEx motor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final PIDFController pidf;
    private boolean emergencyStop = false;
    private double target = 0;
    private double speed = 0;
    private boolean manual = true;

    /**
     * Constructs a Lift with a HardwareMap.
     * @param hwMap the HardwareMap
     */
    public Slide(HardwareMap hwMap) {
        motor = new MotorEx(hwMap, "slide", Motor.GoBILDA.RPM_435);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(KP, KI, KD);
        motor.resetEncoder();
        pidf = new PIDFController(KP, KI, KD, KF);
        pidf.setTolerance(TE, TD);
        pidf.setSetPoint(0);
    }

    /* **** Commands: **** */

    /**
     * Should be called in a loop to set the motor power.
     */
    public void update() {
        sendTelemetry();
        if (manual) {
            motor.set(speedMod * speed);
            return;
        }
        double speedTarget = pidf.calculate(motor.getCurrentPosition(), target);
        if (emergencyStop || pidf.atSetPoint()) {
            motor.stopMotor();
        } else {
            motor.set(speedMod * speedTarget);
        }
        dashboardTelemetry.addData("PIDF calc target", speedTarget);
    }

    public boolean isAtTarget() {
        return pidf.atSetPoint();
    }

    /**
     * An "emergency stop" if tuning makes the lift go crazy
     * Toggle forces the lift to 0 power
     */
    public void toggleStop() {
        emergencyStop = !emergencyStop;
    }

    public void setSpeed(double speed) {
        speed = Range.clip(speed, -1, 1);
        if (this.speed == speed) return;
        manual = true;
        dashboardTelemetry.addData("Slide Speed", speed);
        dashboardTelemetry.update();
        this.speed = speed;
    }

    /**
     * Set the height of the lift.
     * @param pos the position to set
     */
    public void setHeight(double pos) {
        manual = false;
        // Range.clip protects from inadvertently setting it too high or too low
        target = Range.clip(pos, 0, MAX_LENGTH);
        pidf.setPIDF(KP, KI, KD, KF);
        pidf.setTolerance(TE, TD);
        pidf.setSetPoint(target);
    }

    /**
     * Sends telemetry data for tuning/debugging purposes. Can be graphed with FTC Dashboard
     * which is pretty nifty
     * The FTC Dashboard address is 192.168.43.1:8080/dash
     */
    public void sendTelemetry() {
        int pos = motor.getCurrentPosition();
        dashboardTelemetry.addData("Slide Position (mm)", ticksToMm(pos));
        dashboardTelemetry.addData("Slide Target (mm)", ticksToMm(target));
        dashboardTelemetry.addData("Slide Error(mm)", ticksToMm(pos - target));
        dashboardTelemetry.addData("Slide Velocity", motor.getVelocity()); //Ticks/second?
        dashboardTelemetry.update();
    }

    public static double mmToTicks(double mm) {
        return mm * TICKS_PER_MM;
    }

    public static double ticksToMm(double ticks) {
        return ticks / TICKS_PER_MM;
    }
}
