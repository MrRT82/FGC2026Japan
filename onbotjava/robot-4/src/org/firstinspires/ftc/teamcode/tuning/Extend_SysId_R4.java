package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import java.io.*;
import java.util.*;

/**
 * Robot 4 only: open-loop stick/slip identification data for tools/rebuild_extend_model.py.
 * Writes the CSV, a same-stem .properties snapshot and a same-stem _events.csv.
 */
@TeleOp(name="Extend_SysId_R4", group="1_DIAG")
public final class Extend_SysId_R4 extends LinearOpMode {
    public static final String BUILD = "20260927_robot4_extend_sysid_v1_nolimits";
    private static final long PERIOD_NS = 20_000_000L;

    private static void acquire(ExtendSysIdLog.Reading r, DcMotorEx motor, List<LynxModule> hubs,
                                int index, long origin) {
        long start = System.nanoTime();
        r.value = Double.NaN; r.ok = false;
        try {
            if (index == 0) r.value = motor.getCurrent(CurrentUnit.AMPS);
            else {
                LynxModule hub = hubs.get((index - 1) / 2);
                r.value = index % 2 == 1 ? hub.getInputVoltage(VoltageUnit.VOLTS) : hub.getCurrent(CurrentUnit.AMPS);
            }
            r.ok = ExtendTuneConfig.finite(r.value) && r.value >= 0;
        } catch (RuntimeException ignored) { /* Failure is NaN + ok=0, never zero current. */ }
        long end = System.nanoTime();
        r.stampMs = (end - origin) / 1e6; r.durationMs = (end - start) / 1e6; r.seq++;
    }


    /** Hub that powers the extension motor: configured index, else matched by connection info. */
    private static int motorHub(ExtendSysIdConfig c, DcMotorEx motor, List<LynxModule> hubs) throws IOException {
        if (c.motorHubIndex >= 0) {
            if (c.motorHubIndex >= hubs.size()) throw new IOException("motor_hub_index out of range");
            return c.motorHubIndex;
        }
        String info = "";
        try { info = motor.getController().getConnectionInfo(); } catch (RuntimeException ignored) { }
        for (int n = 0; n < hubs.size(); n++) if (hubs.get(n).getConnectionInfo().equals(info)) return n;
        if (hubs.size() == 1) return 0;
        throw new IOException("Cannot identify extension motor Hub; set motor_hub_index");
    }

    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx motor = null;
        TuneCsvWriter writer = null, events = null;
        ExtendSysIdCore core = null;
        long origin = System.nanoTime();
        try {
            ExtendSysIdConfig config = ExtendSysIdConfig.load(new File(ExtendSysIdConfig.PATH));
            // Do not call RobotHardware.init(): it moves unrelated servos and resets encoders.
            for (DcMotor other : hardwareMap.getAll(DcMotor.class)) other.setPower(0);
            motor = hardwareMap.get(DcMotorEx.class, "extendmotor");
            motor.setPower(0);
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AnalogInput pot = hardwareMap.get(AnalogInput.class, "potentiometer");
            double maxV = pot.getMaxVoltage();
            if (!(maxV > 0)) throw new IOException("Invalid potentiometer max voltage");
            List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
            if (hubs.isEmpty()) throw new IOException("No Hub electrical measurements available");
            int motorHub = motorHub(config, motor, hubs);
            int motorHubVoltage = 1 + 2 * motorHub;
            ExtendSysIdLog.Reading[] readings = new ExtendSysIdLog.Reading[1 + 2 * hubs.size()];
            for (int n = 0; n < readings.length; n++) readings[n] = new ExtendSysIdLog.Reading();
            int[] others = new int[readings.length - 1];
            for (int n = 0, k = 0; n < readings.length; n++) if (n != motorHubVoltage) others[k++] = n;
            List<String> hubInfo = new ArrayList<>();
            for (LynxModule hub : hubs) hubInfo.add(hub.getConnectionInfo());
            File directory = new File("/sdcard/FIRST/logs");
            if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create log directory");
            String stem = "robot4_extend_sysid_" + System.currentTimeMillis() + "_" + Long.toHexString(origin);
            File csv = new File(directory, stem + ".csv");
            ExtendSysIdLog.metadata(new File(directory, stem + ".properties"), BUILD, config, motorHub, maxV, hubInfo);
            writer = new TuneCsvWriter(csv, ExtendSysIdLog.header(hubs.size()));
            events = new TuneCsvWriter(new File(directory, stem + "_events.csv"), ExtendSysIdCore.EVENT_HEADER);
            core = new ExtendSysIdCore(config);
            boolean wasA = false;
            long loop = 0, previousLoop = 0, lastTelemetry = 0;
            int otherIndex = 0;
            while (!isStopRequested()) {
                long begin = System.nanoTime();
                double period = previousLoop == 0 ? 0 : (begin - previousLoop) / 1e6;
                previousLoop = begin;
                // Release/abort before potentially blocking SDK calls.
                if (!isStarted() || !gamepad1.right_bumper || gamepad1.b) motor.setPower(0);
                if (gamepad1.b) core.trip("OPERATOR_ABORT");
                // Averaging a few reads lowers potentiometer noise; the timestamp is the read midpoint.
                long angleStart = System.nanoTime();
                double potV = Double.NaN, angle = Double.NaN;
                try {
                    double sum = 0;
                    for (int n = 0; n < config.angleSamples; n++) sum += pot.getVoltage();
                    potV = sum / config.angleSamples;
                    angle = potV / maxV * 135.0;
                } catch (RuntimeException ignored) { }
                long angleEnd = System.nanoTime();
                double angleReadMs = (angleEnd - angleStart) / 1e6;
                double angleTMs = ((angleStart - origin) + (angleEnd - origin)) / 2e6;
                // One electrical SDK call per loop: the motor-Hub voltage every other loop, since
                // duty = effort_V / voltage; the remaining signals in rotation on the other loops.
                int readIndex = loop % 2 == 0 ? motorHubVoltage : others.length == 0 ? motorHubVoltage : others[otherIndex++ % others.length];
                acquire(readings[readIndex], motor, hubs, readIndex, origin);
                loop++;
                long now = System.nanoTime();
                double nowMs = (now - origin) / 1e6;
                String powerFault = angleReadMs > 100 || now - angleEnd > 100_000_000L ? "ANGLE_READ_DELAY" : "";
                boolean active = isStarted() && !isStopRequested();
                boolean deadman = active && gamepad1.right_bumper && !gamepad1.b;
                boolean start = gamepad1.a && !wasA;
                ExtendSysIdLog.Reading hubVolts = readings[motorHubVoltage];
                core.update(angle, angleTMs, deadman, start, hubVolts.ok ? hubVolts.value : Double.NaN,
                            hubVolts.age(nowMs), powerFault, writer.healthy() && events.healthy());
                wasA = gamepad1.a;
                if (isStopRequested() || !gamepad1.right_bumper || gamepad1.b) core.release();
                motor.setPower(core.sent);
                double outputTMs = (System.nanoTime() - origin) / 1e6;
                String row = ExtendSysIdLog.row(nowMs, active, core, potV, maxV, angleReadMs, angleTMs, period,
                                                outputTMs, deadman, config.balls, writer.pending(), writer.lastSyncMs, readings);
                if (!writer.offer(row)) { core.trip("LOGGER_FAILED"); motor.setPower(0); }
                while (!core.finishedEvents.isEmpty())
                    if (!events.offer(core.finishedEvents.poll())) { core.trip("LOGGER_FAILED"); motor.setPower(0); }
                if (now - lastTelemetry >= 200_000_000L) {
                    telemetry.addData("Build", BUILD);
                    telemetry.addData("Mode", config.motionEnabled
                        ? "RB held + A: automatic up/down events; release RB: pause; B: latch stop"
                        : "MONITOR ONLY - motion_enabled=false");
                    telemetry.addData("State", core.state + " " + core.fault + " " + core.endReason);
                    telemetry.addData("Event", "%d/%d %s dir %d scale %.2f", core.event + 1, config.maxEvents,
                                      core.phase, core.direction, core.scale);
                    telemetry.addData("Electrical", powerFault);
                    telemetry.addData("Angle / origin / goal", "%.2f / %.2f / %.2f", angle, core.origin, core.goal);
                    telemetry.addData("Effort V / rest V / duty", "%.2f / %.2f / %.3f", core.commandV, core.restV, core.sent);
                    telemetry.addData("Motor Hub", "%d (%s)", motorHub, ExtendSysIdLog.voltageSignal(motorHub));
                    telemetry.addData("Motor A", readings[0].value);
                    for (int n = 0; n < hubs.size(); n++)
                        telemetry.addData("Hub " + n + " V / A", "%.2f / %.2f  %s", readings[1 + n * 2].value,
                                          readings[2 + n * 2].value, hubInfo.get(n));
                    telemetry.addData("CSV", csv.getName()); telemetry.addData("Logger", writer.error + events.error);
                    telemetry.update(); lastTelemetry = now;
                }
                long remaining = PERIOD_NS - (System.nanoTime() - begin);
                if (remaining > 1_000_000) sleep(remaining / 1_000_000); else idle();
            }
        } catch (Exception e) {
            if (core != null) core.trip("EXCEPTION");
            telemetry.addData("SysId stopped", e.toString()); telemetry.update();
            if (e instanceof InterruptedException) Thread.currentThread().interrupt();
        } finally {
            // Zero power before waiting for disk. A stalled SDK/USB call is not a hard real-time stop.
            try { if (motor != null) motor.setPower(0); }
            finally {
                try {
                    if (core != null && events != null) while (!core.finishedEvents.isEmpty()) events.offer(core.finishedEvents.poll());
                } finally {
                    if (writer != null) writer.close();
                    if (events != null) events.close();
                }
            }
        }
    }
}
