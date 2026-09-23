// ============================================================================
// 最終変更: 2026-09-23 14:48:47 JST (UTC+09:00)
// 変更内容: センサーAPI所要時間と制御周期の計測を追加。取得頻度・制御設定は維持。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

// 元のAPIを1回だけ呼ぶ。例外は記録した後、元の呼出し元へそのまま伝える。
public final class SensorReads {
    public final SensorTiming timing;
    public SensorReads(SensorTiming timing) { this.timing = timing; }
    public double velocity(DcMotorEx motor, int id) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            double value = motor.getVelocity();
            failed = false;
            return value;
        } finally { timing.recordSince(id, started, failed); }
    }
    public double current(DcMotorEx motor, int id) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            double value = motor.getCurrent(CurrentUnit.AMPS);
            failed = false;
            return value;
        } finally { timing.recordSince(id, started, failed); }
    }
    public double hubCurrent(LynxModule hub, int id) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            double value = hub.getCurrent(CurrentUnit.AMPS);
            failed = false;
            return value;
        } finally { timing.recordSince(id, started, failed); }
    }
    public double voltage(LynxModule hub, int id) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            double value = hub.getInputVoltage(VoltageUnit.VOLTS);
            failed = false;
            return value;
        } finally { timing.recordSince(id, started, failed); }
    }
    public double angle(AnalogInput pot, double offset) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            double value = (pot.getVoltage() / pot.getMaxVoltage()) * 270 / 2 - offset;
            failed = false;
            return value;
        } finally { timing.recordSince(SensorTiming.POT_ANGLE, started, failed); }
    }
    public boolean touch(TouchSensor sensor) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            boolean value = sensor.isPressed();
            failed = false;
            return value;
        } finally { timing.recordSince(SensorTiming.TOUCH, started, failed); }
    }
    public double distance(DistanceSensor sensor) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            double value = sensor.getDistance(DistanceUnit.CM);
            failed = false;
            return value;
        } finally { timing.recordSince(SensorTiming.DISTANCE, started, failed); }
    }
    public YawPitchRollAngles heading(IMU imu) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            YawPitchRollAngles value = imu.getRobotYawPitchRollAngles();
            failed = false;
            return value;
        } finally { timing.recordSince(SensorTiming.IMU_YPR, started, failed); }
    }
    public AngularVelocity gyro(IMU imu) {
        long started = System.nanoTime();
        boolean failed = true;
        try {
            AngularVelocity value = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            failed = false;
            return value;
        } finally { timing.recordSince(SensorTiming.IMU_GYRO, started, failed); }
    }
}
