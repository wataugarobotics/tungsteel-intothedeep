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
public class Lift extends SubsystemBase {
    public static double KP = 0.04;
    public static double KI = 0.0001;
    public static double KD = 0.0008;
    public static double KF = 0.0001;
    public static double TE = 50.0;
    public static double TD = 5.0;

    public static double TOP_HEIGHT = mmToTicks(700);

    private static final double TICKS_PER_MM =  384.5 / 112;
    private final double MAX_HEIGHT = mmToTicks(960);

    public static double speedMod = 0.75;

    private final MotorEx motor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final PIDFController pidf;
    private boolean emergencyStop = false;
    private double target = 0;

    /**
     * Constructs a Lift with a HardwareMap.
     * @param hwMap the HardwareMap
     */
    public Lift(HardwareMap hwMap) {
        motor = new MotorEx(hwMap, "lift", Motor.GoBILDA.RPM_435);
        motor.setInverted(true);
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
        double speedTarget = pidf.calculate(motor.getCurrentPosition(), target);
        if (emergencyStop || pidf.atSetPoint()) {
            motor.stopMotor();
        } else {
            motor.set(speedMod * speedTarget);
        }
        dashboardTelemetry.addData("PIDF calc target", speedTarget);
    }

    /**
     * An "emergency stop" if tuning makes the lift go crazy
     * Toggle forces the lift to 0 power
     */
    public void toggleStop() {
        emergencyStop = !emergencyStop;
    }

    /**
     * Set the height of the lift.
     * @param pos the position to set
     */
    public void setHeight(double pos) {
        // Range.clip protects from inadvertently setting it too high or too low
        target = Range.clip(pos, 0, MAX_HEIGHT);
        pidf.setPIDF(KP, KI, KD, KF);
        pidf.setTolerance(TE, TD);
        pidf.setSetPoint(target);
    }

    public void toggle() {
        setHeight(target == 0 ? TOP_HEIGHT : 0);
    }

    public boolean isAtTarget() {
        return pidf.atSetPoint();
    }

    /**
     * Sends telemetry data for tuning/debugging purposes. Can be graphed with FTC Dashboard
     * which is pretty nifty
     * The FTC Dashboard address is 192.168.43.1:8080/dash
     */
    public void sendTelemetry() {
        int pos = motor.getCurrentPosition();
        dashboardTelemetry.addData("Lift Position (mm)", ticksToMm(pos));
        dashboardTelemetry.addData("Lift Target (mm)", ticksToMm(target));
        dashboardTelemetry.addData("Lift Error(mm)", ticksToMm(pos - target));
        dashboardTelemetry.addData("Lift Velocity", motor.getVelocity()); // Ticks/second?
        dashboardTelemetry.update();
    }

    public static double mmToTicks(double mm) {
        return mm * TICKS_PER_MM;
    }

    public static double ticksToMm(double ticks) {
        return ticks / TICKS_PER_MM;
    }
}
