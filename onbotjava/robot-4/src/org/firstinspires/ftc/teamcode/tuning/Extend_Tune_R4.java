package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import java.io.*;
import java.util.*;

/** Robot 4 only: current logging and small supervised extension trials. */
@TeleOp(name="Extend_Tune_R4", group="1_DIAG")
public final class Extend_Tune_R4 extends LinearOpMode {
    public static final String BUILD = "20260927_robot4_extend_tune_v2";
    private static final long PERIOD_NS = 20_000_000L;
    private static final double MAX_AGE_MS = 250;
    private static final class Reading {
        double value = Double.NaN, durationMs;
        long stampNs, seq;
        boolean ok;
        double age(long now) { return stampNs == 0 ? -1 : (now - stampNs) / 1e6; }
        String csv(long now) {
            return "," + value + "," + (ok ? 1 : 0) + "," + seq + "," + age(now) + "," + durationMs;
        }
    }
    private static String readingHeader(String name) {
        return "," + name + "," + name + "_ok," + name + "_seq," + name + "_age_ms," + name + "_read_ms";
    }
    private void acquire(Reading r, DcMotorEx motor, List<LynxModule> hubs, int index) {
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
        r.stampNs = System.nanoTime(); r.durationMs = (r.stampNs - start) / 1e6; r.seq++;
    }
    private static String electricalFault(Reading[] readings, long now, ExtendTuneConfig c) {
        for (int n = 0; n < readings.length; n++) {
            Reading r = readings[n];
            if (!r.ok || r.age(now) < 0 || r.age(now) > MAX_AGE_MS || r.durationMs > 100) return "ELECTRICAL_STALE_OR_INVALID";
            if (n == 0 && r.value > c.motorLimitA) return "MOTOR_CURRENT";
            if (n > 0 && n % 2 == 1 && r.value < c.minVoltage) return "LOW_VOLTAGE_HUB_" + ((n - 1) / 2);
            if (n > 0 && n % 2 == 0 && r.value > c.hubLimitA) return "HUB_CURRENT_" + ((n - 1) / 2);
        }
        return "";
    }
    private static void metadata(File file, ExtendTuneConfig c, List<LynxModule> hubs, DcMotorEx motor) throws IOException {
        try (PrintWriter w = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file), "UTF-8"))) {
            w.println("build=" + BUILD); w.println("robot=4"); w.println("expected_adb_serial=dddad6c23c4eeb3b");
            w.println("motion_enabled=" + c.motionEnabled); w.println("balls=" + c.balls);
            w.println("gear_ratio=" + c.gearRatio); w.println("motor_model=" + c.motorModel);
            w.println("min_angle_deg=" + c.minAngle); w.println("max_angle_deg=" + c.maxAngle);
            w.println("motor_limit_A=" + c.motorLimitA); w.println("hub_limit_A=" + c.hubLimitA);
            w.println("min_voltage_V=" + c.minVoltage); w.println("max_power=" + c.maxPower);
            w.println("kp=" + c.kp); w.println("ki=" + c.ki); w.println("kd=" + c.kd);
            w.println("gravity_cos=" + c.gravityCos); w.println("gravity_constant=" + c.gravityConstant);
            w.println("gravity_phase_deg=" + c.gravityPhaseDeg); w.println("friction=" + c.friction);
            w.println("csv_schema=robot4_extend_tune_v2");
            w.println("trial_mode=" + c.trialMode); w.println("dataset_role=" + c.datasetRole);
            w.println("step_deg=" + c.stepDeg); w.println("slew_dps=" + c.slewDps);
            w.println("motor_hub_index=" + c.motorHubIndex);
            w.println("motor_hub_voltage_signal=" + (c.motorHubIndex < 0 ? "unassigned" : "hub_" + c.motorHubIndex + "_voltage_V"));
            w.println("motor_controller_connection=" + motor.getController().getConnectionInfo());
            w.println("motor_port=" + motor.getPortNumber());
            w.println("ramp_rate_V_per_s=" + c.rampRateVps); w.println("ramp_max_delta_V=" + c.rampMaxDeltaV);
            w.println("ramp_max_excursion_deg=" + c.rampMaxExcursionDeg);
            w.println("ramp_stationary_window_ms=500; max_span_deg=0.15; max_velocity_dps=0.5");
            w.println("ramp_onset_displacement_deg=0.3; confirm_ms=100; post_confirm_ms=150");
            w.println("angle_conversion=voltage/maxVoltage*135; offset=0");
            w.println("zero_power=BRAKE; not a mechanical holding brake");
            for (int n = 0; n < hubs.size(); n++) w.println("hub_" + n + "=" + hubs.get(n).getConnectionInfo());
            if (w.checkError()) throw new IOException("Metadata write failed");
        }
    }
    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx motor = null;
        TuneCsvWriter writer = null;
        ExtendTuneCore core = null;
        long origin = System.nanoTime();
        try {
            // Do not call RobotHardware.init(): it moves unrelated servos and resets encoders.
            for (DcMotor other : hardwareMap.getAll(DcMotor.class)) other.setPower(0);
            ExtendTuneConfig config = ExtendTuneConfig.load(new File(ExtendTuneConfig.PATH));
            motor = hardwareMap.get(DcMotorEx.class, "extendmotor");
            motor.setPower(0);
            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AnalogInput pot = hardwareMap.get(AnalogInput.class, "potentiometer");
            List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
            if (hubs.isEmpty()) throw new IOException("No Hub electrical measurements available");
            if (config.motorHubIndex >= hubs.size()) throw new IOException("motor_hub_index outside connected Hub list");
            Reading[] readings = new Reading[1 + 2 * hubs.size()];
            for (int n = 0; n < readings.length; n++) readings[n] = new Reading();
            File directory = new File("/sdcard/FIRST/logs");
            if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create log directory");
            String stem = "robot4_extend_tune_" + System.currentTimeMillis() + "_" + Long.toHexString(origin);
            File csv = new File(directory, stem + ".csv");
            metadata(new File(directory, stem + ".properties"), config, hubs, motor);
            String header = "t_ms,started,state,fault,angle_deg,pot_V,pot_max_V,angle_read_ms,angle_t_ms,velocity_dps,target_deg,goal_deg,requested_out,sent_out,ff,p,i,d,dt_ms,period_ms,output_t_ms,deadman,balls,writer_pending,last_sync_ms";
            header += readingHeader("motor_current_A");
            for (int n = 0; n < hubs.size(); n++) header += readingHeader("hub_" + n + "_voltage_V") + readingHeader("hub_" + n + "_current_A");
            header += ",trial_id,trial_mode,dataset_role,phase,event,stop_reason,trial_ms,baseline_V,ramp_delta_V,command_V,motor_hub_index,start_edge,target_step,last_still_t_ms,first_motion_t_ms";
            writer = new TuneCsvWriter(csv, header);
            core = new ExtendTuneCore(config);
            boolean wasA = false, wasUp = false, wasDown = false;
            int readIndex = 0;
            long previousLoop = 0, lastTelemetry = 0;
            while (!isStopRequested()) {
                long begin = System.nanoTime();
                double period = previousLoop == 0 ? 0 : (begin - previousLoop) / 1e6;
                previousLoop = begin;
                // Release/abort before potentially blocking SDK calls.
                if (!isStarted() || !gamepad1.right_bumper || gamepad1.b) motor.setPower(0);
                if (gamepad1.b) core.trip("OPERATOR_ABORT");
                long angleStart = System.nanoTime();
                double potV = Double.NaN, maxV = Double.NaN, angle = Double.NaN;
                try {
                    potV = pot.getVoltage(); maxV = pot.getMaxVoltage();
                    if (maxV > 0) angle = potV / maxV * 135.0;
                } catch (RuntimeException ignored) { }
                long angleEndNs = System.nanoTime();
                double angleReadMs = (angleEndNs - angleStart) / 1e6;
                // One electrical SDK call per loop; values have independent ages and sequences.
                acquire(readings[readIndex], motor, hubs, readIndex);
                readIndex = (readIndex + 1) % readings.length;
                long now = System.nanoTime();
                String powerFault = electricalFault(readings, now, config);
                if (angleReadMs > 100 || now - angleEndNs > 100_000_000L) powerFault = "ANGLE_READ_DELAY";
                boolean active = isStarted() && !isStopRequested();
                boolean deadman = active && gamepad1.right_bumper && !gamepad1.b;
                boolean start = gamepad1.a && !wasA;
                int step = (gamepad1.dpad_up && !wasUp ? 1 : 0) - (gamepad1.dpad_down && !wasDown ? 1 : 0);
                double selectedVoltage = config.motorHubIndex < 0 ? Double.NaN : readings[1 + 2 * config.motorHubIndex].value;
                core.update(angle, now, deadman, start, step, powerFault, writer.healthy(), selectedVoltage);
                wasA = gamepad1.a; wasUp = gamepad1.dpad_up; wasDown = gamepad1.dpad_down;
                if (isStopRequested() || !gamepad1.right_bumper || gamepad1.b) core.release();
                motor.setPower(core.sent);
                long outputNs = System.nanoTime();
                StringBuilder row = new StringBuilder(768);
                row.append((now - origin) / 1e6).append(',').append(active ? 1 : 0).append(',').append(core.state)
                    .append(',').append(core.fault).append(',').append(angle).append(',').append(potV).append(',').append(maxV)
                    .append(',').append(angleReadMs).append(',').append((angleEndNs - origin) / 1e6).append(',').append(core.velocity).append(',').append(core.target).append(',').append(core.goal)
                    .append(',').append(core.requested).append(',').append(core.sent).append(',').append(core.ff)
                    .append(',').append(core.p).append(',').append(core.i).append(',').append(core.d)
                    .append(',').append(core.dt * 1000).append(',').append(period).append(',').append((outputNs - origin) / 1e6)
                    .append(',').append(deadman ? 1 : 0).append(',').append(config.balls).append(',').append(writer.pending()).append(',').append(writer.lastSyncMs);
                for (Reading r : readings) row.append(r.csv(now));
                row.append(',').append(core.trialId).append(',').append(config.trialMode).append(',').append(config.datasetRole)
                    .append(',').append(core.phase).append(',').append(core.event).append(',').append(core.stopReason)
                    .append(',').append(core.trialMs).append(',').append(core.baselineV).append(',').append(core.rampDeltaV)
                    .append(',').append(core.commandV).append(',').append(config.motorHubIndex)
                    .append(',').append(start ? 1 : 0).append(',').append(step)
                    .append(',').append(core.lastStillNs == 0 ? -1 : (core.lastStillNs - origin) / 1e6)
                    .append(',').append(core.firstMotionNs == 0 ? -1 : (core.firstMotionNs - origin) / 1e6);
                if (!writer.offer(row.toString())) { core.trip("LOGGER_FAILED"); motor.setPower(0); }
                if (now - lastTelemetry >= 200_000_000L) {
                    telemetry.addData("Build", BUILD);
                    telemetry.addData("Mode", config.motionEnabled ? config.trialMode + ": RB held + A; B: latch stop" : "MONITOR ONLY - motion limits not enabled");
                    telemetry.addData("State", core.state + " " + core.fault);
                    telemetry.addData("Trial / phase", core.trialId + " " + core.phase + " " + core.stopReason);
                    telemetry.addData("Dataset", config.datasetRole + " balls=" + config.balls);
                    telemetry.addData("Motor Hub index (verify)", config.motorHubIndex);
                    telemetry.addData("Motor controller", motor.getController().getConnectionInfo());
                    telemetry.addData("Ramp baseline / delta V", "%.3f / %.3f", core.baselineV, core.rampDeltaV);
                    telemetry.addData("Electrical", powerFault);
                    telemetry.addData("Angle / target", "%.2f / %.2f", angle, core.target);
                    telemetry.addData("Motor A", readings[0].value);
                    for (int n = 0; n < hubs.size(); n++) {
                        telemetry.addData("Hub " + n + " V / A", "%.2f / %.2f", readings[1 + n * 2].value, readings[2 + n * 2].value);
                        telemetry.addData("Hub " + n + " connection", hubs.get(n).getConnectionInfo());
                    }
                    telemetry.addData("CSV", csv.getName()); telemetry.addData("Logger", writer.error);
                    telemetry.update(); lastTelemetry = now;
                }
                long remaining = PERIOD_NS - (System.nanoTime() - begin);
                if (remaining > 1_000_000) sleep(remaining / 1_000_000); else idle();
            }
        } catch (Exception e) {
            if (core != null) core.trip("EXCEPTION");
            telemetry.addData("Tune stopped", e.toString()); telemetry.update();
            if (e instanceof InterruptedException) Thread.currentThread().interrupt();
        } finally {
            // Zero power before waiting for disk. A stalled SDK/USB call is not a hard real-time stop.
            try { if (motor != null) motor.setPower(0); }
            finally { if (writer != null) writer.close(); }
        }
    }
}
