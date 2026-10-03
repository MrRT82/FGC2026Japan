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
import java.util.Map;
import java.util.HashMap;

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
        final Map<String, Double> outputs = new HashMap<>();
        RemainingHardware() { super(null, null); }
        @Override public <T> T get(Class<? extends T> type, String name) {
            if (!names.contains(name)) throw new IllegalArgumentException("Missing device: " + name);
            requested.add(name);
            if (type == AnalogInput.class) return type.cast(new AnalogInput(null, 0));
            return type.cast(Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type},
                    (proxy, method, args) -> {
                        if (method.getName().equals("setPosition") || method.getName().equals("setPower"))
                            outputs.put(name, (Double) args[0]);
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
        check(map.outputs.get("rightLock") == 0.5 && map.outputs.get("leftLock") == 0.5,
                "Continuous rotation locks must initialize stopped");
        ClimbSequence sequence = new ClimbSequence();
        check(sequence.update(1, 0, 75) == ClimbSequence.LOWER && sequence.limitsExtend() && !sequence.hasRisen(),
                "gp1 left trigger first lowers the extension; rise waits");
        check(sequence.update(0, 400, 65) == ClimbSequence.LOWER, "Rise waits while the extension is above 61");
        check(sequence.update(1, 800, 62) == ClimbSequence.LOWER, "Press while lowering is ignored");
        sequence.update(0, 900, 61.5);
        check(sequence.update(0, 1000, 60.8) == ClimbSequence.UP && sequence.hasRisen(),
                "Rise starts once the extension is at or below 61");
        check(sequence.update(1, 5000, 60) == ClimbSequence.UP, "Press during the rise is ignored");
        sequence.update(0, 5100, 60);
        check(sequence.update(1, 11_600, 60) == ClimbSequence.DOWN, "Press after the rise starts descent");
        sequence.update(0, 11_700, 60);
        check(sequence.update(1, 17_000, 60) == ClimbSequence.DOWN, "Press before the roller starts is ignored");
        sequence.update(0, 17_100, 60);
        check(sequence.update(1, 17_700, 60) == ClimbSequence.HOLD, "Press after the roller starts stops and holds");
        sequence.update(0, 17_800, 60);
        check(sequence.update(1, 18_200, 60) == ClimbSequence.ROLL, "Next press spins the roller again");
        sequence.update(0, 18_300, 60);
        check(sequence.update(1, 18_700, 60) == ClimbSequence.HOLD, "Presses keep alternating");
        ClimbSequence low = new ClimbSequence();
        check(low.update(1, 0, 20) == ClimbSequence.UP, "Extension already low: rise starts at once");
        ClimbSequence noPot = new ClimbSequence();
        check(noPot.update(1, 0, Double.NaN) == ClimbSequence.LOWER, "Unreadable extension angle does not start the rise");        ClimbController climb = new ClimbController(hw);
        climb.update(ClimbSequence.LOWER, 0, 0, 16, 16, 1.0);
        check(map.outputs.get("rightLock") == 0.5 && map.outputs.get("rightInside") == 0.5
                && map.outputs.get("climb") == 0.0, "Servos and roller stay stopped while lowering the extension");
        climb.update(ClimbSequence.UP, 0, 100, 16, 16, 1.0);
        check(map.outputs.get("rightLock") == 1.0 && map.outputs.get("rightInside") == 0.5
                && map.outputs.get("climb") == 0.0, "Rise rotates locks only");
        climb.update(ClimbSequence.UP, 10_600, 10_600, 16, 16, 1.0);
        check(map.outputs.get("rightLock") == 0.5, "Rise stops after UP_MS");
        climb.update(ClimbSequence.DOWN, 20_000, 0, 16, 16, 1.0);
        check(map.outputs.get("rightInside") == 0.0 && map.outputs.get("rightLock") == 0.0
                && map.outputs.get("climb") == 0.0, "Descent turns climb servos and locks; roller waits");
        climb.update(ClimbSequence.DOWN, 26_000, 6_000, 16, 16, 1.0);
        check(map.outputs.get("climb") == 1.0 && map.outputs.get("rightInside") == 0.0,
                "Roller runs at full power 6 s after the descent press while still descending");
        climb.update(ClimbSequence.HOLD, 26_500, 0, 16, 16, 1.0);
        check(map.outputs.get("climb") == 0.0 && climb.isHolding() && map.outputs.get("rightInside") == 0.0,
                "Stop holds the roller position; descent continues to DOWN_MS");
        climb.update(ClimbSequence.HOLD, 27_100, 600, 16, 16, 1.0);
        check(map.outputs.get("rightInside") == 0.5, "Descent stops at DOWN_MS");
        climb.update(ClimbSequence.ROLL, 27_500, 0, 196, 16, 1.0);
        check(map.outputs.get("climb") == 1.0, "Roller direction stays fixed after the first spin");
        new ClimbController(hw).update(ClimbSequence.DOWN, 0, 6_000, -16, -16, -1.0);
        check(map.outputs.get("climb") == -1.0, "Red front roller direction is reversed");
        new ClimbController(hw).update(ClimbSequence.DOWN, 0, 6_000, 164, -16, -1.0);
        check(map.outputs.get("climb") == 1.0, "Red back side reverses the roller");        DriveController drive = new DriveController(hw);
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
