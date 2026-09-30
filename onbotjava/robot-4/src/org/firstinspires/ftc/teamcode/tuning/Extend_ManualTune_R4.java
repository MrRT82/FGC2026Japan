package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.io.*;

/** Robot 4 interactive PID tuning with the same FF law as Extend_TuneAuto_R4. */
@TeleOp(name="Extend_ManualTune_R4", group="1_DIAG")
public final class Extend_ManualTune_R4 extends LinearOpMode {
    public static final String BUILD = "20260930_robot4_extend_manual_v1_nolimits";
    private static final long PERIOD_NS = 20_000_000L;
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx motor = null;
        TuneCsvWriter writer = null;
        try {
            ExtendTuneConfig c = ExtendTuneConfig.load(new File(ExtendTuneConfig.PATH));
            for (DcMotor other : hardwareMap.getAll(DcMotor.class)) other.setPower(0);
            motor = hardwareMap.get(DcMotorEx.class, "extendmotor");
            motor.setPower(0); motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AnalogInput pot = hardwareMap.get(AnalogInput.class, "potentiometer");
            File dir = new File("/sdcard/FIRST/logs");
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create log directory");
            String stem = "robot4_extend_manual_" + System.currentTimeMillis();
            writer = new TuneCsvWriter(new File(dir, stem + ".csv"),
                "t_ms,state,event,angle_deg,velocity_dps,target_deg,goal_deg,kp,ki,kd,ff,p,i,d,requested_out,sent_out,motor_current_A,deadman");
            try (PrintWriter meta = new PrintWriter(new File(dir, stem + ".properties"), "UTF-8")) {
                meta.println("build=" + BUILD); meta.println("kp_start=" + c.kp); meta.println("ki=" + c.ki);
                meta.println("kd_start=" + c.kd); meta.println("gravity_cos=" + c.gravityCos);
                meta.println("gravity_constant=" + c.gravityConstant); meta.println("gravity_phase_deg=" + c.gravityPhaseDeg);
                meta.println("friction=" + c.friction); meta.println("auto_kv_up=" + c.autoKvUp);
                meta.println("auto_kv_down=" + c.autoKvDown); meta.println("slew_dps=" + c.autoSlewDps);
                meta.println("limits=none; output clamped only to +/-1");
            }

            double kp = c.kp, ki = c.ki, kd = c.kd;
            double angle = 0, previousAngle = 0, velocity = 0, target = 0, goal = 0;
            double previousError = 0, integral = 0;
            long origin = System.nanoTime(), previousNs = 0, lastTelemetry = 0;
            boolean armed = false, stopped = false;
            boolean wasA=false, wasB=false, wasUp=false, wasDown=false, wasLeft=false, wasRight=false, wasX=false, wasY=false;
            while (!isStopRequested()) {
                long begin = System.nanoTime();
                double maxV = pot.getMaxVoltage(), potV = pot.getVoltage();
                angle = maxV > 0 ? potV / maxV * 135.0 : Double.NaN;
                double dt = previousNs == 0 ? .020 : (begin - previousNs) / 1e9;
                if (!ExtendTuneConfig.finite(angle) || dt < .002 || dt > .100) { stopped = true; armed = false; }
                if (previousNs == 0) { previousAngle = angle; target = goal = angle; }
                else velocity += .35 * ((angle - previousAngle) / dt - velocity);
                previousAngle = angle; previousNs = begin;

                boolean a = gamepad1.a && !wasA, b = gamepad1.b && !wasB;
                boolean up = gamepad1.dpad_up && !wasUp, down = gamepad1.dpad_down && !wasDown;
                boolean left = gamepad1.dpad_left && !wasLeft, right = gamepad1.dpad_right && !wasRight;
                boolean x = gamepad1.x && !wasX, y = gamepad1.y && !wasY;
                String event = "";
                if (b) { stopped = true; armed = false; event = "LATCH_STOP"; }
                if (a && !stopped) { armed = true; target = goal = angle; integral = previousError = 0; event = "ARM"; }
                if (armed) {
                    if (up) { goal += 2; event = "TARGET_UP"; }
                    if (down) { goal -= 2; event = "TARGET_DOWN"; }
                    if (right) { kd = clamp(kd + .00025, 0, .02); event = "KD_UP"; }
                    if (left) { kd = clamp(kd - .00025, 0, .02); event = "KD_DOWN"; }
                    if (y) { kp = clamp(kp + .005, 0, .5); event = "KP_UP"; }
                    if (x) { kp = clamp(kp - .005, 0, .5); event = "KP_DOWN"; }
                }
                boolean deadman = isStarted() && gamepad1.right_bumper && !stopped;
                double ff=0,p=0,i=0,d=0,requested=0,sent=0;
                if (armed && deadman) {
                    double oldTarget = target;
                    target += clamp(goal - target, -c.autoSlewDps * dt, c.autoSlewDps * dt);
                    double targetVelocity = (target - oldTarget) / dt;
                    double error = target - angle;
                    integral = clamp(integral + error * dt, -.5, .5);
                    p = kp * error; i = ki * integral; d = kd * (error - previousError) / dt; previousError = error;
                    double friction = velocity * Math.signum(error) > 15 ? 0 : c.friction * clamp(error / 2, -1, 1);
                    ff = c.gravityCos * Math.cos(Math.toRadians(angle + c.gravityPhaseDeg)) + c.gravityConstant + friction
                       + c.autoKvUp * Math.max(0, targetVelocity) + c.autoKvDown * Math.min(0, targetVelocity);
                    requested = ff + p + i + d; sent = clamp(requested, -1, 1);
                } else { integral = 0; previousError = 0; }
                motor.setPower(sent);
                double current;
                try { current = motor.getCurrent(CurrentUnit.AMPS); } catch (RuntimeException e) { current = Double.NaN; }
                writer.offer(((begin-origin)/1e6)+","+(stopped?"STOPPED":armed?"ACTIVE":"MONITOR")+","+event+","+angle+","+velocity+","+target+","+goal+","+kp+","+ki+","+kd+","+ff+","+p+","+i+","+d+","+requested+","+sent+","+current+","+(deadman?1:0));
                if (begin - lastTelemetry > 150_000_000L) {
                    telemetry.addData("Build", BUILD); telemetry.addData("State", stopped?"STOPPED":armed?"ACTIVE":"MONITOR");
                    telemetry.addData("Controls", "RB deadman; A arm; Up/Down target; Left/Right Kd; X/Y Kp; B stop");
                    telemetry.addData("Angle / target / goal", "%.2f / %.2f / %.2f", angle,target,goal);
                    telemetry.addData("Kp / Ki / Kd", "%.5f / %.5f / %.5f", kp,ki,kd);
                    telemetry.addData("Out = FF + P + I + D", "%.3f = %.3f + %.3f + %.3f + %.3f", sent,ff,p,i,d);
                    telemetry.addData("Motor A / CSV", "%.3f / %s", current,stem+".csv"); telemetry.update(); lastTelemetry=begin;
                }
                wasA=gamepad1.a; wasB=gamepad1.b; wasUp=gamepad1.dpad_up; wasDown=gamepad1.dpad_down;
                wasLeft=gamepad1.dpad_left; wasRight=gamepad1.dpad_right; wasX=gamepad1.x; wasY=gamepad1.y;
                long remain=PERIOD_NS-(System.nanoTime()-begin); if(remain>1_000_000)sleep(remain/1_000_000);else idle();
            }
        } catch (Exception e) { telemetry.addData("Manual tune stopped", e.toString()); telemetry.update(); }
        finally { try { if(motor!=null) motor.setPower(0); } finally { if(writer!=null) writer.close(); } }
    }
}
