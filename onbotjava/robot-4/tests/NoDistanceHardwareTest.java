package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import java.lang.reflect.Proxy;
import java.lang.reflect.Field;
import java.io.BufferedWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

// Runs against SDK stubs with only the hardware that remains on Robot 4.
public final class NoDistanceHardwareTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }
    private static final class RemainingHardware extends HardwareMap {
        final Set<String> names = new HashSet<>(Arrays.asList(
                "leftmotor", "rightmotor", "left_shooter", "right_shooter",
                "intakemotor", "extendmotor", "feed", "climb", "rightInside", "rightOutside",
                "leftInside", "leftOutside", "rightLock", "leftLock", "potentiometer",
                "imu", "rightTouch", "leftTouch"));
        final Set<String> requested = new HashSet<>();
        RemainingHardware() { super(null, null); }
        @Override public <T> T get(Class<? extends T> type, String name) {
            if (!names.contains(name)) throw new IllegalArgumentException("Missing device: " + name);
            requested.add(name);
            if (type == AnalogInput.class) return type.cast(new AnalogInput(null, 0));
            return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type},
                    (proxy, method, args) -> {
                        Class<?> result = method.getReturnType();
                        if (result == boolean.class) return true;
                        if (result == double.class) return 0.0;
                        if (result == float.class) return 0.0f;
                        if (result == int.class) return 0;
                        return null;
                    }));
        }
    }
    private static void set(Object target, String name, Object value) throws Exception {
        Field f = target.getClass().getDeclaredField(name);
        f.setAccessible(true);
        f.set(target, value);
    }
    public static void main(String[] args) throws Exception {
        RemainingHardware map = new RemainingHardware();
        RobotHardware hw = new RobotHardware();
        hw.init(map);
        check(map.requested.equals(map.names), "Production hardware must initialize with remaining devices only");
        RemainingHardware timingMap = new RemainingHardware();
        new org.firstinspires.ftc.teamcode.timing.RobotHardware().init(timingMap);
        check(timingMap.requested.equals(timingMap.names), "Timing hardware must initialize without the removed sensor");
        DriveController drive = new DriveController(hw);
        for (double forward : new double[]{-1, -0.4, 0, 0.4, 1}) {
            for (double turn : new double[]{-1, -0.2, 0, 0.2, 1}) {
                DriveController.DriveOutput output = drive.driveWithImuCorrection(forward, turn);
                double f = Math.signum(forward) * Math.pow(Math.abs(forward), 1.2);
                double scale = Math.max(1, Math.max(Math.abs(f-turn), Math.abs(f+turn)));
                check(Math.abs(output.leftPower - (f-turn)/scale) < 1e-10, "Manual forward plus heading correction: left");
                check(Math.abs(output.rightPower - (f+turn)/scale) < 1e-10, "Manual forward plus heading correction: right");
            }
        }
        ShooterController shooter = new ShooterController(hw);
        ExtendController extend = new ExtendController(hw);
        RobotLogger logger = new RobotLogger(hw, null, new ImuController((IMU)null), extend, shooter);
        StringWriter sink = new StringWriter();
        BufferedWriter out = new BufferedWriter(sink);
        set(logger, "out", out);
        Gamepad gp = new Gamepad();
        logger.update(gp, gp, 0, 0, true, false, false, 0);
        out.flush();
        check(!logger.isFailed(), "Production CSV must write without removed sensor");
        String header = new String(Files.readAllBytes(Paths.get(args[0])), StandardCharsets.UTF_8).trim();
        check(sink.toString().trim().split(",", -1).length == header.split(",", -1).length,
                "Production CSV row and header column counts must agree");
        System.out.println("NoDistanceHardwareTest passed: " + checks + " checks; fake hardware only");
    }
}
