package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import java.io.BufferedWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.concurrent.atomic.AtomicInteger;

// 実機なしで集計窓・別スレッド・API呼出し回数と例外伝播を検証する。
public final class SensorTimingTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }
    private static long[] drain(SensorTiming timing) {
        StringBuilder sb = new StringBuilder();
        timing.appendWindow(sb);
        String[] fields = sb.substring(1).split(",");
        check(fields.length == SensorTiming.csvHeader().substring(1).split(",").length,
                "CSV header and data must have equal column counts");
        long[] values = new long[fields.length];
        for (int i = 0; i < fields.length; i++) values[i] = Long.parseLong(fields[i]);
        return values;
    }
    private static void set(Object target, String name, Object value) throws Exception {
        Field field = target.getClass().getDeclaredField(name);
        field.setAccessible(true);
        field.set(target, value);
    }
    private static void loggerCheck(DcMotorEx motor) throws Exception {
        RobotHardware hw = new RobotHardware();
        hw.leftMotor = hw.rightMotor = hw.left_shooterMotor = hw.right_shooterMotor = motor;
        hw.intakeMotor = hw.extendMotor = motor;
        ShooterController shooter = new ShooterController(hw);
        ExtendController extend = new ExtendController(hw);
        RobotLogger logger = new RobotLogger(hw, null, new ImuController((IMU) null), extend, shooter);
        StringWriter sink = new StringWriter();
        BufferedWriter out = new BufferedWriter(sink);
        set(logger, "out", out);
        hw.timing.start();
        Gamepad gp = new Gamepad();
        logger.update(gp, gp, 0, 0, false, false, false, 0);
        check(!logger.isFailed(), "CSV write must succeed");
        set(logger, "lastLogMs", 0L);
        logger.update(gp, gp, 0, 0, true, false, false, 0);
        out.flush();
        String[] rows = sink.toString().trim().split("\\n");
        String[] header = RobotLogger.csvHeader().trim().split(",");
        check(rows.length == 2, "Two complete CSV rows expected");
        String[] second = rows[1].split(",");
        check(rows[0].split(",").length == header.length && second.length == header.length,
                "Complete robot CSV rows must match the header including all timing fields");
        for (int i = 0; i < header.length; i++) {
            if (header[i].equals("tm_log_write_n")) check(second[i].equals("1"), "Write time belongs to the next window");
            if (header[i].equals("tm_log_flush_n")) check(second[i].equals("1"), "Flush time belongs to the next window");
            if (header[i].equals("tm_velocity_shooter_n")) check(second[i].equals("2"), "Logger must retain both shooter readings");
        }
    }
    public static void main(String[] args) throws Exception {
        SensorTiming timing = new SensorTiming();
        timing.record(SensorTiming.IMU_YPR, 1000, 2000, false);
        check(drain(timing)[0] == 0, "Initialization must not contaminate runtime measurements");
        timing.startAt(1000);
        timing.record(SensorTiming.IMU_YPR, 999, 5000, false);
        timing.record(SensorTiming.IMU_YPR, 1000, 2000, false);
        timing.record(SensorTiming.IMU_YPR, 2000, 72000, true);
        long[] values = drain(timing);
        check(values[0] == 2 && values[1] == 71 && values[2] == 70 && values[3] == 1,
                "Count, total, spike and error must survive between CSV rows");
        check(drain(timing)[0] == 0, "A drained window must not repeat old samples");

        Thread producer = new Thread(() -> {
            for (int i = 0; i < 100000; i++) timing.record(SensorTiming.TOUCH, 1000, 4000, i % 2 == 0);
        });
        producer.start();
        long count = 0, total = 0, errors = 0;
        do {
            values = drain(timing);
            count += values[SensorTiming.TOUCH * 4];
            total += values[SensorTiming.TOUCH * 4 + 1];
            errors += values[SensorTiming.TOUCH * 4 + 3];
        } while (producer.isAlive());
        producer.join();
        values = drain(timing);
        count += values[SensorTiming.TOUCH * 4];
        total += values[SensorTiming.TOUCH * 4 + 1];
        errors += values[SensorTiming.TOUCH * 4 + 3];
        check(count == 100000 && total == 300000 && errors == 50000,
                "Concurrent export must neither lose nor duplicate readings");

        SensorTiming live = new SensorTiming();
        SensorReads reads = new SensorReads(live);
        live.start();
        AtomicInteger calls = new AtomicInteger();
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[] {DcMotorEx.class}, (proxy, method, arguments) -> {
                    if (method.getName().equals("getVelocity")) { calls.incrementAndGet(); return 123.5; }
                    throw new AssertionError("Unexpected hardware call: " + method.getName());
                });
        check(reads.velocity(motor, SensorTiming.VELOCITY_SHOOTER) == 123.5 && calls.get() == 1,
                "Instrumentation must preserve the value and read exactly once");
        RuntimeException expected = new IllegalStateException("simulated sensor failure");
        TouchSensor sensor = (TouchSensor) Proxy.newProxyInstance(TouchSensor.class.getClassLoader(),
                new Class<?>[] {TouchSensor.class}, (proxy, method, arguments) -> { throw expected; });
        try { reads.touch(sensor); throw new AssertionError("Exception was swallowed"); }
        catch (RuntimeException actual) { check(actual == expected, "Original exception must propagate"); }
        values = drain(live);
        check(values[SensorTiming.TOUCH * 4] == 1 && values[SensorTiming.TOUCH * 4 + 3] == 1,
                "Failed sensor calls must be counted and timed");
        check(values[SensorTiming.VELOCITY_SHOOTER * 4] == 1,
                "Successful hardware calls must be counted");
        loggerCheck(motor);
        System.out.println("SensorTimingTest passed: " + checks + " checks");
    }
}
