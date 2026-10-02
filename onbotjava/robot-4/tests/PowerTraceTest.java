package org.firstinspires.ftc.teamcode.timing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.lang.reflect.*;
import java.nio.file.*;
import java.util.*;

public final class PowerTraceTest {
    public static void main(String[] args) throws Exception {
        final int[] calls = {0};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(), new Class[]{DcMotorEx.class},
            (proxy, method, values) -> {
                if (method.getName().equals("getCurrent")) {
                    calls[0]++;
                    if (calls[0] == 2) throw new IllegalStateException("synthetic failure");
                    return 1.25;
                }
                return null;
            });
        RobotHardware hw = new RobotHardware(); hw.extendMotor = motor;
        ExtendController ext = new ExtendController(hw);
        RobotLogger logger = new RobotLogger(hw, null, null, ext, null);
        Path dir = Files.createTempDirectory("r4-power-test");
        PowerTrace trace = new PowerTrace(dir.toFile());
        Field field = RobotLogger.class.getDeclaredField("powerTrace"); field.setAccessible(true); field.set(logger, trace);
        Method current = RobotLogger.class.getDeclaredMethod("cur", DcMotorEx.class, int.class); current.setAccessible(true);
        if ((double) current.invoke(logger, motor, SensorTiming.DIAG_EXTEND) != 1.25) throw new AssertionError("value changed");
        if ((double) current.invoke(logger, motor, SensorTiming.DIAG_EXTEND) != 0) throw new AssertionError("legacy error changed");
        logger.close();
        List<String> lines = Files.readAllLines(Paths.get(trace.path));
        if (calls[0] != 2 || lines.size() != 3) throw new AssertionError("extra reads or rows");
        if (!lines.get(1).contains(",extend_A,1.25,1,") || !lines.get(2).contains(",extend_A,NaN,0,")) throw new AssertionError("failure hidden");
        for (String row : lines) if (row.split(",", -1).length != 16) throw new AssertionError("CSV width");
        System.out.println("PowerTraceTest passed: existing API count, success/failure rows, CSV, drain");
    }
}
