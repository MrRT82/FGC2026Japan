package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.*;
import java.util.Locale;

/**
 * Robot 4 only: checks the measured climb times without the match program.
 *
 * Press gamepad1 A once: up for UP_MS, then stops. Press A again: down for DOWN_MS, then
 * stops. Further presses alternate. Outputs, names and directions match
 * RobotHardware/ClimbController (up: lock only at full speed; down: climb and lock at full speed).
 * Times are from Climb_TimeMeasure_R4 (2026-10-03: up 10.8 / 11.6 s, down 7.6 / 8.9 s).
 *
 * Press X: climb roller motor "climb" at ROLLER_POWER (-1.0, reverse); press X again to stop.
 * Its start time is recorded (seconds since START and since the last climb move ended).
 * B stops everything; the next A repeats the same direction.
 * Log: /sdcard/FIRST/logs/robot4_climb_timed_*.csv
 */
@TeleOp(name="Climb_TimedTest_R4", group="1_DIAG")
public final class Climb_TimedTest_R4 extends LinearOpMode {
    public static final String BUILD = "20261003_robot4_climb_timed_v2";
    private static final long UP_MS = 10_500, DOWN_MS = 7_000;
    // Same as ClimbController: 0.5 stop.
    private static final double STOP = 0.5, CLIMB_UP = 0.5, CLIMB_DOWN = 0.0;
    private static final double LOCK_UP = 1.0, LOCK_DOWN = 0.0;
    private static final double ROLLER_POWER = -1.0;

    private Servo rightUp, rightDown, leftUp, leftDown, rightLock, leftLock;
    private DcMotor roller;
    private PrintWriter log;
    private long originMs;

    private void set(double climb, double lock) {
        rightUp.setPosition(climb); rightDown.setPosition(climb);
        leftUp.setPosition(climb); leftDown.setPosition(climb);
        rightLock.setPosition(lock); leftLock.setPosition(lock);
    }

    private void record(long now, String event, String detail) {
        if (log == null) return;
        log.printf(Locale.US, "%.3f,%s,%s%n", (now - originMs) / 1000.0, event, detail);
        log.flush();
    }

    @Override public void runOpMode() throws InterruptedException {
        // Same names and directions as RobotHardware (2026-10-01).
        rightUp = hardwareMap.get(Servo.class, "rightInside");
        rightDown = hardwareMap.get(Servo.class, "rightOutside");
        leftUp = hardwareMap.get(Servo.class, "leftInside");
        leftDown = hardwareMap.get(Servo.class, "leftOutside");
        rightLock = hardwareMap.get(Servo.class, "rightLock");
        leftLock = hardwareMap.get(Servo.class, "leftLock");
        roller = hardwareMap.get(DcMotor.class, "climb");
        rightUp.setDirection(Servo.Direction.REVERSE);
        rightDown.setDirection(Servo.Direction.FORWARD);
        leftUp.setDirection(Servo.Direction.FORWARD);
        leftDown.setDirection(Servo.Direction.REVERSE);
        rightLock.setDirection(Servo.Direction.FORWARD);
        leftLock.setDirection(Servo.Direction.REVERSE);
        set(STOP, STOP);
        roller.setPower(0);

        File dir = new File("/sdcard/FIRST/logs");
        if (!dir.isDirectory()) dir.mkdirs();
        File csv = new File(dir, "robot4_climb_timed_" + System.currentTimeMillis() + ".csv");
        try {
            log = new PrintWriter(new OutputStreamWriter(new FileOutputStream(csv), "UTF-8"));
            log.println("t_since_start_s,event,detail"); log.flush();
        } catch (IOException e) { log = null; }

        telemetry.addData("Build", BUILD);
        telemetry.addData("Use", "A: up %.1f s / next A: down %.1f s. X: roller %.1f on/off. B: stop all",
                          UP_MS / 1000.0, DOWN_MS / 1000.0, ROLLER_POWER);
        telemetry.update();
        waitForStart();
        originMs = System.currentTimeMillis();
        record(originMs, "START", "");

        boolean nextUp = true, running = false, rollerOn = false, wasA = false, wasX = false;
        long startMs = 0, limitMs = 0, lastMoveEndMs = -1, rollerStartMs = -1;
        String last = "-", rollerInfo = "-";
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            boolean a = gamepad1.a, pressedA = a && !wasA;
            boolean x = gamepad1.x, pressedX = x && !wasX;
            wasA = a; wasX = x;

            if (gamepad1.b) {
                if (running) {
                    set(STOP, STOP); running = false;
                    last = (nextUp ? "UP" : "DOWN") + " stopped by B at " + (now - startMs) + " ms";
                    record(now, nextUp ? "UP_STOP_B" : "DOWN_STOP_B", Long.toString(now - startMs));
                }
                if (rollerOn) {
                    roller.setPower(0); rollerOn = false;
                    record(now, "ROLLER_STOP_B", Long.toString(now - rollerStartMs));
                }
            } else {
                if (pressedA && !running) {
                    running = true; startMs = now;
                    limitMs = nextUp ? UP_MS : DOWN_MS;
                    if (nextUp) set(CLIMB_UP, LOCK_UP); else set(CLIMB_DOWN, LOCK_DOWN);
                    record(now, nextUp ? "UP_START" : "DOWN_START", Long.toString(limitMs));
                } else if (running && now - startMs >= limitMs) {
                    set(STOP, STOP); running = false; lastMoveEndMs = now;
                    last = (nextUp ? "UP" : "DOWN") + " done " + limitMs + " ms";
                    record(now, nextUp ? "UP_DONE" : "DOWN_DONE", Long.toString(limitMs));
                    nextUp = !nextUp;
                }
                if (pressedX) {
                    if (!rollerOn) {
                        roller.setPower(ROLLER_POWER); rollerOn = true; rollerStartMs = now;
                        String afterMove = lastMoveEndMs < 0 ? "no move yet"
                            : String.format(Locale.US, "%.2f s after last move", (now - lastMoveEndMs) / 1000.0);
                        rollerInfo = String.format(Locale.US, "started at %.2f s (%s)", (now - originMs) / 1000.0, afterMove);
                        record(now, "ROLLER_START", afterMove);
                    } else {
                        roller.setPower(0); rollerOn = false;
                        record(now, "ROLLER_STOP", Long.toString(now - rollerStartMs));
                    }
                }
            }

            telemetry.addData("Build", BUILD);
            telemetry.addData("Time since START", "%.2f s", (now - originMs) / 1000.0);
            telemetry.addData("Climb", running ? (nextUp ? "UP running" : "DOWN running")
                                               : (nextUp ? "press A: UP" : "press A: DOWN"));
            if (running) telemetry.addData("Elapsed", "%d / %d ms", now - startMs, limitMs);
            telemetry.addData("Last climb", last);
            telemetry.addData("Roller", rollerOn ? String.format(Locale.US, "ON %.1f (%d ms)", ROLLER_POWER, now - rollerStartMs) : "off");
            telemetry.addData("Roller start", rollerInfo);
            telemetry.addData("CSV", csv.getName());
            telemetry.update();
            idle();
        }
        set(STOP, STOP);
        roller.setPower(0);
        if (log != null) { record(System.currentTimeMillis(), "STOP", ""); log.close(); }
    }
}
