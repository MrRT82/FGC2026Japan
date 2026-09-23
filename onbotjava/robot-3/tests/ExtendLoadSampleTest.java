package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

public final class ExtendLoadSampleTest {
    private static void check(boolean value, String message) {
        if (!value) throw new AssertionError(message);
    }
    private static Map<String, String> row(ExtendLoadSample sample) {
        StringBuilder sb = new StringBuilder();
        sample.append(sb);
        String[] keys = ExtendLoadSample.csvHeader().substring(1).split(",");
        String[] values = sb.substring(1).split(",");
        check(keys.length == values.length, "Load CSV alignment");
        Map<String, String> result = new HashMap<>();
        for (int i = 0; i < keys.length; i++) result.put(keys[i], values[i]);
        return result;
    }
    public static void main(String[] args) throws Exception {
        AtomicInteger calls = new AtomicInteger();
        boolean[] fail = {false};
        RobotHardware hw = new RobotHardware();
        hw.extendMotor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, arguments) -> {
                    check(method.getName().equals("getCurrent"), "No motor writes or extra sensor reads");
                    calls.incrementAndGet();
                    if (fail[0]) throw new IllegalStateException("offline");
                    return 2.75;
                });
        ExtendController ext = new ExtendController(hw);
        ext.current = 32;
        ext.observedControlTarget = 40;
        ext.poteVel = 10;
        ext.output = .4;
        ext.observationNs = System.nanoTime();
        ExtendLoadSample sample = new ExtendLoadSample();
        sample.initialBalls = 4;
        hw.timing.start();
        sample.sample(hw.reads, ext, 12, 20, 25);
        Map<String, String> first = row(sample);
        check(calls.get() == 1, "One current call per sample");
        check(first.get("load_current_A").equals("2.75"), "Current preserved");
        check(first.get("load_initial_balls").equals("4"), "Manual label preserved");
        check(first.get("load_current_ok").equals("1"), "Valid sample flagged");
        Field last = ExtendLoadSample.class.getDeclaredField("lastNs");
        last.setAccessible(true);
        long lastTime = last.getLong(sample);
        check(!sample.due(lastTime + 99_999_999L), "Rate limited below 100ms");
        check(sample.due(lastTime + 100_000_000L), "Due at 100ms");
        last.setLong(sample, System.nanoTime() + 1_000_000_000L);
        ext.current = 99;
        sample.sample(hw.reads, ext, 11, 50, 30);
        check(calls.get() == 1 && row(sample).equals(first), "Cached sample must retain matching state");
        last.setLong(sample, System.nanoTime() - 200_000_000L);
        fail[0] = true;
        sample.sample(hw.reads, ext, 11, 50, 30);
        Map<String, String> failed = row(sample);
        check(calls.get() == 2, "Failed call attempted once");
        check(failed.get("load_current_A").equals("NaN") && failed.get("load_current_ok").equals("0"),
                "Failure cannot look like zero current or reuse successful reading");
        check(failed.get("load_seq").equals("2"), "Attempt has a unique id even on failure");
        check(ext.output == .4 && ext.poteVel == 10, "Measurement must not change control state");
        System.out.println("ExtendLoadSampleTest passed");
    }
}
