package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.DriveController;
import org.firstinspires.ftc.teamcode.ImuController;
import java.io.*;
import java.util.*;

/** Robot 4 only: supervised drive acceleration, braking and IMU heading trials after a gearbox change. */
@TeleOp(name="Drive_Tune_R4", group="1_DIAG")
public final class Drive_Tune_R4 extends LinearOpMode {
    public static final String BUILD = "20261007_robot4_drive_tune_v1";
    private static final long PERIOD_NS = 20_000_000L;
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
    /** 0/1: left/right drive current; then per Hub voltage, current. */
    private static void acquire(Reading r, int index, DcMotorEx left, DcMotorEx right, List<LynxModule> hubs) {
        long start = System.nanoTime();
        r.value = Double.NaN; r.ok = false;
        try {
            if (index == 0) r.value = left.getCurrent(CurrentUnit.AMPS);
            else if (index == 1) r.value = right.getCurrent(CurrentUnit.AMPS);
            else {
                LynxModule hub = hubs.get((index - 2) / 2);
                r.value = index % 2 == 0 ? hub.getInputVoltage(VoltageUnit.VOLTS) : hub.getCurrent(CurrentUnit.AMPS);
            }
            r.ok = DriveTuneConfig.finite(r.value) && r.value >= 0;
        } catch (RuntimeException ignored) { /* Failure is NaN + ok=0, never zero current. */ }
        r.stampNs = System.nanoTime(); r.durationMs = (r.stampNs - start) / 1e6; r.seq++;
    }
    private static double maxTicksPerSecond(DcMotorEx motor) {
        try { return motor.getMotorType().getAchieveableMaxTicksPerSecond(); }
        catch (RuntimeException e) { return Double.NaN; }
    }
    private static void motor(PrintWriter w, String side, DcMotorEx m) {
        w.println(side + "_controller=" + m.getController().getConnectionInfo());
        w.println(side + "_port=" + m.getPortNumber());
        try {
            MotorConfigurationType t = m.getMotorType();
            w.println(side + "_motor_type=" + t.getName());
            w.println(side + "_ticks_per_rev=" + t.getTicksPerRev());
            w.println(side + "_max_rpm=" + t.getMaxRPM());
            w.println(side + "_achievable_max_ticks_per_s=" + t.getAchieveableMaxTicksPerSecond());
        } catch (RuntimeException e) { w.println(side + "_motor_type_error=" + e); }
        try {
            PIDFCoefficients p = m.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            w.println(side + "_velocity_pidf=" + p.p + "," + p.i + "," + p.d + "," + p.f + "," + p.algorithm);
        } catch (RuntimeException e) { w.println(side + "_velocity_pidf_error=" + e); }
    }
    private static void metadata(File file, DriveTuneConfig c, ImuController imu, List<LynxModule> hubs,
                                 DcMotorEx left, DcMotorEx right) throws IOException {
        try (PrintWriter w = new PrintWriter(new OutputStreamWriter(new FileOutputStream(file), "UTF-8"))) {
            w.println("build=" + BUILD); w.println("robot=4"); w.println("expected_adb_serial=dddad6c23c4eeb3b");
            w.println("csv_schema=robot4_drive_tune_v1");
            w.println("motion_enabled=" + c.motionEnabled);
            w.println("gear_ratio=" + c.gearRatio); w.println("motor_model=" + c.motorModel);
            w.println("gear_reduction=" + c.gearReduction); w.println("wheel_diameter_mm=" + c.wheelDiameterMm);
            w.println("counts_per_motor_rev=" + c.countsPerMotorRev); w.println("metres_per_tick=" + c.metresPerTick());
            w.println("max_power=" + c.maxPower); w.println("max_travel_m=" + c.maxTravelM);
            w.println("max_turn_deg=" + c.maxTurnDeg); w.println("run_ms=" + c.runMs);
            w.println("brake_timeout_ms=" + c.brakeTimeoutMs); w.println("heading_ms=" + c.headingMs);
            w.println("accel_per_s=" + c.accelPerSec); w.println("brake_per_s=" + c.brakePerSec);
            w.println("powers=" + DriveTuneConfig.text(c.powers));
            w.println("heading_steps_deg=" + DriveTuneConfig.text(c.headingSteps));
            w.println("imu_kp=" + imu.kp); w.println("imu_ki=" + imu.ki); w.println("imu_kd=" + imu.kd);
            w.println("imu_ks=" + imu.kS); w.println("imu_kv=" + imu.kV);
            w.println("imu_max_turn_power=" + imu.maxTurnPower); w.println("imu_tolerance_deg=" + imu.tolerance);
            w.println("imu_use_profile=" + imu.useProfile); w.println("imu_turn_slew_per_s=" + imu.turnSlewPerSec);
            w.println("match_brake_strength=" + DriveController.BRAKE_STRENGTH);
            w.println("match_stop_threshold=" + DriveController.STOP_THRESHOLD);
            motor(w, "left", left); motor(w, "right", right);
            w.println("run_mode=RUN_USING_ENCODER; command is a fraction of achievable max motor ticks/s; 0 makes the Hub velocity loop brake");
            w.println("zero_power=BRAKE");
            w.println("imu_orientation=logo LEFT, usb BACKWARD (same as RobotHardware)");
            w.println("turn_sign=left=-u,right=+u; positive u turns toward increasing yaw");
            for (int n = 0; n < hubs.size(); n++) w.println("hub_" + n + "=" + hubs.get(n).getConnectionInfo());
            if (w.checkError()) throw new IOException("Metadata write failed");
        }
    }
    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx left = null, right = null;
        TuneCsvWriter writer = null;
        DriveTuneCore core = null;
        long origin = System.nanoTime();
        try {
            // Do not call RobotHardware.init(): it moves the climb servos and locks.
            for (DcMotor other : hardwareMap.getAll(DcMotor.class)) other.setPower(0);
            DriveTuneConfig config = DriveTuneConfig.load(new File(DriveTuneConfig.PATH));
            left = hardwareMap.get(DcMotorEx.class, "leftmotor");
            right = hardwareMap.get(DcMotorEx.class, "rightmotor");
            // Same direction, zero-power behaviour and run mode as RobotHardware.init().
            left.setDirection(DcMotor.Direction.FORWARD);
            right.setDirection(DcMotor.Direction.REVERSE);
            for (DcMotorEx m : new DcMotorEx[]{left, right}) {
                m.setPower(0);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            IMU imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
            ImuController heading = new ImuController(imu);
            if (DriveTuneConfig.finite(config.imuKp)) heading.kp = config.imuKp;
            if (DriveTuneConfig.finite(config.imuKd)) heading.kd = config.imuKd;
            if (DriveTuneConfig.finite(config.imuKs)) heading.kS = config.imuKs;
            if (DriveTuneConfig.finite(config.imuMaxTurnPower)) heading.maxTurnPower = config.imuMaxTurnPower;
            heading.reset();
            List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
            if (hubs.isEmpty()) throw new IOException("No Hub electrical measurements available");
            for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            Reading[] readings = new Reading[2 + 2 * hubs.size()];
            for (int n = 0; n < readings.length; n++) readings[n] = new Reading();
            File directory = new File("/sdcard/FIRST/logs");
            if (!directory.isDirectory() && !directory.mkdirs()) throw new IOException("Cannot create log directory");
            String stem = "robot4_drive_tune_" + System.currentTimeMillis() + "_" + Long.toHexString(origin);
            File csv = new File(directory, stem + ".csv");
            metadata(new File(directory, stem + ".properties"), config, heading, hubs, left, right);
            String header = "t_ms,started,state,fault,trial_id,trial,phase,event,stop_reason,direction,brake,power,step_deg,"
                + "trial_ms,command,left_out,right_out,left_ticks,right_ticks,left_tps,right_tps,travel_m,speed_mps,"
                + "heading_deg,turned_deg,yaw_rate_dps,pitch_deg,roll_deg,imu_target,imu_out,imu_p,imu_d,imu_s,"
                + "sensor_t_ms,dt_ms,period_ms,output_t_ms,deadman,writer_pending,last_sync_ms";
            header += readingHeader("left_current_A") + readingHeader("right_current_A");
            for (int n = 0; n < hubs.size(); n++) header += readingHeader("hub_" + n + "_voltage_V") + readingHeader("hub_" + n + "_current_A");
            writer = new TuneCsvWriter(csv, header);
            core = new DriveTuneCore(config);
            double mpt = config.metresPerTick();
            double maxTps = maxTicksPerSecond(left);
            boolean wasA = false, wasUp = false, wasDown = false, wasLeft = false, wasRight = false, wasX = false, wasY = false;
            int motorIndex = 0, hubIndex = 2;
            long previousLoop = 0, lastTelemetry = 0;
            while (!isStopRequested()) {
                long begin = System.nanoTime();
                double period = previousLoop == 0 ? 0 : (begin - previousLoop) / 1e6;
                previousLoop = begin;
                // Release/abort before potentially blocking SDK calls.
                if (!isStarted() || !gamepad1.right_bumper || gamepad1.b) { left.setPower(0); right.setPower(0); }
                if (gamepad1.b) core.trip("OPERATOR_ABORT");
                for (LynxModule hub : hubs) hub.clearBulkCache();
                double lt = left.getCurrentPosition(), rt = right.getCurrentPosition();
                double lv = left.getVelocity(), rv = right.getVelocity();
                heading.beginLoop();
                double yaw = Double.NaN, imuOut = 0;
                try { yaw = heading.getHeading(); imuOut = heading.update(); }
                catch (RuntimeException e) { yaw = Double.NaN; imuOut = 0; }
                long sensorNs = System.nanoTime();
                // Two electrical SDK calls per loop: alternate drive sides, cycle Hub values.
                acquire(readings[motorIndex], motorIndex, left, right, hubs);
                motorIndex = 1 - motorIndex;
                acquire(readings[hubIndex], hubIndex, left, right, hubs);
                hubIndex = hubIndex + 1 >= readings.length ? 2 : hubIndex + 1;
                long now = System.nanoTime();
                boolean active = isStarted() && !isStopRequested();
                boolean deadman = active && gamepad1.right_bumper && !gamepad1.b;
                boolean start = gamepad1.a && !wasA;
                core.select((gamepad1.dpad_right && !wasRight ? 1 : 0) - (gamepad1.dpad_left && !wasLeft ? 1 : 0),
                            (gamepad1.dpad_up && !wasUp ? 1 : 0) - (gamepad1.dpad_down && !wasDown ? 1 : 0),
                            gamepad1.x && !wasX, gamepad1.y && !wasY);
                wasA = gamepad1.a; wasUp = gamepad1.dpad_up; wasDown = gamepad1.dpad_down;
                wasLeft = gamepad1.dpad_left; wasRight = gamepad1.dpad_right; wasX = gamepad1.x; wasY = gamepad1.y;
                core.update(now, deadman, start, writer.healthy(), lt, rt, lv, rv, yaw, imuOut);
                boolean release = isStopRequested() || !gamepad1.right_bumper || gamepad1.b;
                left.setPower(release ? 0 : core.left);
                right.setPower(release ? 0 : core.right);
                if (core.wantsImu()) heading.setTarget(core.headingTarget);
                long outputNs = System.nanoTime();
                StringBuilder row = new StringBuilder(900);
                row.append((now - origin) / 1e6).append(',').append(active ? 1 : 0).append(',').append(core.state)
                    .append(',').append(core.fault).append(',').append(core.trialId).append(',').append(DriveTuneCore.TRIALS[core.trial])
                    .append(',').append(core.phase).append(',').append(core.event).append(',').append(core.stopReason)
                    .append(',').append(core.direction).append(',').append(DriveTuneCore.BRAKES[core.brake])
                    .append(',').append(core.power()).append(',').append(core.step()).append(',').append(core.trialMs)
                    .append(',').append(core.command).append(',').append(release ? 0 : core.left).append(',').append(release ? 0 : core.right)
                    .append(',').append(lt).append(',').append(rt).append(',').append(lv).append(',').append(rv)
                    .append(',').append(core.travelM).append(',').append((lv + rv) / 2 * mpt)
                    .append(',').append(yaw).append(',').append(core.turnedDeg).append(',').append(heading.getRate())
                    .append(',').append(heading.getPitch()).append(',').append(heading.getRoll())
                    .append(',').append(core.wantsImu() ? core.headingTarget : Double.NaN).append(',').append(imuOut)
                    .append(',').append(heading.pTerm).append(',').append(heading.dTerm).append(',').append(heading.sTerm)
                    .append(',').append((sensorNs - origin) / 1e6).append(',').append(heading.getDt() * 1000)
                    .append(',').append(period).append(',').append((outputNs - origin) / 1e6)
                    .append(',').append(deadman ? 1 : 0).append(',').append(writer.pending()).append(',').append(writer.lastSyncMs);
                for (Reading r : readings) row.append(r.csv(now));
                if (!writer.offer(row.toString())) { core.trip("LOGGER_FAILED"); left.setPower(0); right.setPower(0); }
                if (now - lastTelemetry >= 200_000_000L) {
                    telemetry.addData("ビルド", BUILD);
                    telemetry.addData("モード", config.motionEnabled
                        ? "RBを押したままAで開始 / RBを離すと停止 / Bでラッチ停止" : "計測のみ（motion_enabled=false）");
                    telemetry.addData("状態", core.state + " " + core.fault + " " + core.stopReason);
                    telemetry.addData("試験", "%s  %s  向き%s  ブレーキ:%s", DriveTuneCore.TRIALS[core.trial],
                        core.trial == 2 ? core.step() + "度" : "出力" + core.power(),
                        core.direction > 0 ? "+" : "-", core.brakeText());
                    telemetry.addData("選択", "十字左右:試験 上下:出力/角度 X:向き Y:ブレーキ");
                    telemetry.addData("試行 / 段階", core.trialId + " / " + core.phase);
                    telemetry.addData("距離m / 旋回deg", "%.2f / %.1f", core.travelM, core.turnedDeg);
                    telemetry.addData("速度m/s 左/右", "%.2f / %.2f", lv * mpt, rv * mpt);
                    telemetry.addData("方位 / 角速度", "%.1f / %.0f", yaw, heading.getRate());
                    telemetry.addData("ピッチ / ロール", "%.1f / %.1f", heading.getPitch(), heading.getRoll());
                    telemetry.addData("出力1.0の速度m/s", "%.2f (最大%.0f ticks/s)", maxTps * mpt, maxTps);
                    telemetry.addData("電流A 左/右", "%.2f / %.2f", readings[0].value, readings[1].value);
                    for (int n = 0; n < hubs.size(); n++)
                        telemetry.addData("Hub " + n + " V / A", "%.2f / %.2f", readings[2 + n * 2].value, readings[3 + n * 2].value);
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
            try { if (left != null) left.setPower(0); }
            finally {
                try { if (right != null) right.setPower(0); }
                finally { if (writer != null) writer.close(); }
            }
        }
    }
}
