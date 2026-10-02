package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.*;
import java.util.*;

/**
 * Robot 4 only, temporary: measures how long the climb must run up and down, to set
 * ClimbController.CLIMB_UP_MS / CLIMB_DOWN_MS.
 *
 * Hold gamepad1 A: runs (first press up, next press down, alternating). Release: stops and
 * records the held time. Outputs, names and directions match RobotHardware/ClimbController
 * (up: lock only at full speed; down: climb and lock at full speed). B: stop and ignore input until
 * released. Log: /sdcard/FIRST/logs/robot4_climb_time_*.csv
 */
@TeleOp(name="Climb_TimeMeasure_R4", group="1_DIAG")
public final class Climb_TimeMeasure_R4 extends LinearOpMode {
    public static final String BUILD = "20260930_robot4_climb_time_v5";
    // Same as ClimbController: 0.5 stop. Up turns only the lock at full speed (climb held
    // stopped); down turns the climb and the lock at full speed.
    private static final double STOP = 0.5, CLIMB_UP = 0.5, CLIMB_DOWN = 0.0;
    private static final double LOCK_UP = 1.0, LOCK_DOWN = 0.0;

    private Servo rightUp, rightDown, leftUp, leftDown, rightLock, leftLock;

    private void set(double climb, double lock) {
        rightUp.setPosition(climb); rightDown.setPosition(climb);
        leftUp.setPosition(climb); leftDown.setPosition(climb);
        rightLock.setPosition(lock); leftLock.setPosition(lock);
    }

    @Override public void runOpMode() throws InterruptedException {
        // Same names and directions as RobotHardware (2026-09-30).
        rightUp = hardwareMap.get(Servo.class, "rightInside");
        rightDown = hardwareMap.get(Servo.class, "rightOutside");
        leftUp = hardwareMap.get(Servo.class, "leftInside");
        leftDown = hardwareMap.get(Servo.class, "leftOutside");
        rightLock = hardwareMap.get(Servo.class, "rightLock");
        leftLock = hardwareMap.get(Servo.class, "leftLock");
        rightUp.setDirection(Servo.Direction.REVERSE);
        rightDown.setDirection(Servo.Direction.FORWARD);
        leftUp.setDirection(Servo.Direction.FORWARD);
        leftDown.setDirection(Servo.Direction.REVERSE);
        rightLock.setDirection(Servo.Direction.FORWARD);
        leftLock.setDirection(Servo.Direction.REVERSE);
        set(STOP, STOP);

        File dir = new File("/sdcard/FIRST/logs");
        if (!dir.isDirectory()) dir.mkdirs();
        File csv = new File(dir, "robot4_climb_time_" + System.currentTimeMillis() + ".csv");
        PrintWriter log = null;
        try { log = new PrintWriter(new OutputStreamWriter(new FileOutputStream(csv), "UTF-8")); }
        catch (IOException e) { telemetry.addData("Log", "cannot open " + csv + ": " + e); }
        if (log != null) { log.println("index,direction,held_ms"); log.flush(); }

        telemetry.addData("Build", BUILD);
        telemetry.addData("Use", "Hold A: up / next hold: down (alternating). Release: stop + record. B: stop");
        telemetry.update();
        waitForStart();

        boolean up = true, running = false, blocked = false;
        long startNs = 0;
        int index = 0;
        List<String> results = new ArrayList<>();
        double lastUp = Double.NaN, lastDown = Double.NaN;
        while (opModeIsActive()) {
            boolean a = gamepad1.a, b = gamepad1.b;
            long now = System.nanoTime();
            if (b) { if (running) { set(STOP, STOP); running = false; } blocked = true; }
            else if (!a) blocked = false;

            if (!running && a && !blocked) {
                running = true; startNs = now;
                if (up) set(CLIMB_UP, LOCK_UP); else set(CLIMB_DOWN, LOCK_DOWN);
            } else if (running && !a) {
                set(STOP, STOP);
                running = false;
                double ms = (now - startNs) / 1e6;
                index++;
                String name = up ? "UP" : "DOWN";
                results.add(String.format(Locale.US, "%d %s %.0f ms", index, name, ms));
                if (up) lastUp = ms; else lastDown = ms;
                if (log != null) { log.printf(Locale.US, "%d,%s,%.1f%n", index, name, ms); log.flush(); }
                up = !up;
            }

            telemetry.addData("Build", BUILD);
            telemetry.addData("Next", running ? (up ? "UP running" : "DOWN running") : (up ? "hold A: UP" : "hold A: DOWN"));
            if (running) telemetry.addData("Held", "%.0f ms", (now - startNs) / 1e6);
            telemetry.addData("Last UP / DOWN", "%.0f / %.0f ms", lastUp, lastDown);
            for (int n = Math.max(0, results.size() - 8); n < results.size(); n++) telemetry.addLine(results.get(n));
            telemetry.addData("CSV", csv.getName());
            telemetry.update();
            idle();
        }
        set(STOP, STOP);
        if (log != null) log.close();
    }
}
