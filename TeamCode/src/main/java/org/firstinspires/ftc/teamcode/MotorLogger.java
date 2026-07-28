package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;v
import java.util.Locale;

public class MotorLogger {

    // ===== 定数 =====
    private static final String DIR            = "/sdcard/FIRST/logs/";
    private static final int    LOG_INTERVAL_MS = 50; // 20Hz

    // ===== モーター =====
    private final DcMotorEx  leftMotor;
    private final DcMotorEx  rightMotor;
    private final DcMotorEx  leftShooter;
    private final DcMotorEx  rightShooter;
    private final DcMotorEx  intakeMotor;
    private final HardwareMap hardwareMap;

    // ===== ファイル =====
    private FileWriter driveVelWriter;
    private FileWriter driveCurWriter;
    private FileWriter shooterVelWriter;
    private FileWriter shooterCurWriter;
    private FileWriter intakeVelWriter;
    private FileWriter intakeCurWriter;
    private FileWriter hubVoltWriter;
    private final String timestamp;

    // ===== タイミング =====
    private long startTime   = 0;
    private long lastLogTime = 0;

    // =============================================
    // コンストラクタ
    // =============================================

    public MotorLogger(RobotHardware hardware, HardwareMap hwMap) {
        leftMotor    = hardware.leftMotor;
        rightMotor   = hardware.rightMotor;
        leftShooter  = hardware.left_shooterMotor;
        rightShooter = hardware.right_shooterMotor;
        intakeMotor  = (DcMotorEx) hardware.intakeMotor;
        hardwareMap  = hwMap;
        timestamp    = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
    }

    // =============================================
    // 初期化
    // =============================================

    public void init() {
        startTime   = System.currentTimeMillis();
        lastLogTime = 0;

        try {
            File dir = new File(DIR);
            if (!dir.exists()) dir.mkdirs();

            driveVelWriter   = openWriter("drive_velocity",   "time_ms,left_vel,right_vel");
            driveCurWriter   = openWriter("drive_current",    "time_ms,left_current_A,right_current_A");
            shooterVelWriter = openWriter("shooter_velocity", "time_ms,avg_vel");
            shooterCurWriter = openWriter("shooter_current",  "time_ms,avg_current_A");
            intakeVelWriter  = openWriter("intake_velocity",  "time_ms,vel");
            intakeCurWriter  = openWriter("intake_current",   "time_ms,current_A");
            hubVoltWriter    = openWriter("hub_voltage",      "time_ms,voltage_V");

        } catch (IOException e) {
            closeAll();
        }
    }

    private FileWriter openWriter(String baseName, String header) throws IOException {
        String fileName = baseName + "_" + timestamp + ".csv";
        File   file     = new File(DIR + fileName);
        FileWriter fw   = new FileWriter(file, false);
        fw.append(header + "\n");
        fw.flush();
        return fw;
    }

    // =============================================
    // 更新（メインループから毎ループ呼ぶ）
    // =============================================

    public void update() {
        long now = System.currentTimeMillis();
        if (now - lastLogTime < LOG_INTERVAL_MS) return;
        lastLogTime = now;

        long elapsed = now - startTime;

        try {
            // ===== ドライブ速度 =====
            if (driveVelWriter != null) {
                driveVelWriter.append(String.format(Locale.US,
                        "%d,%.2f,%.2f\n",
                        elapsed,
                        leftMotor.getVelocity(),
                        rightMotor.getVelocity()
                ));
                driveVelWriter.flush();
            }

            // ===== ドライブ電流 =====
            if (driveCurWriter != null) {
                driveCurWriter.append(String.format(Locale.US,
                        "%d,%.3f,%.3f\n",
                        elapsed,
                        leftMotor.getCurrent(CurrentUnit.AMPS),
                        rightMotor.getCurrent(CurrentUnit.AMPS)
                ));
                driveCurWriter.flush();
            }

            // ===== シューター速度（平均）=====
            if (shooterVelWriter != null) {
                double avgVel = (leftShooter.getVelocity()
                        + rightShooter.getVelocity()) / 2.0;
                shooterVelWriter.append(String.format(Locale.US,
                        "%d,%.2f\n",
                        elapsed,
                        avgVel
                ));
                shooterVelWriter.flush();
            }

            // ===== シューター電流（平均）=====
            if (shooterCurWriter != null) {
                double avgCur = (leftShooter.getCurrent(CurrentUnit.AMPS)
                        + rightShooter.getCurrent(CurrentUnit.AMPS)) / 2.0;
                shooterCurWriter.append(String.format(Locale.US,
                        "%d,%.3f\n",
                        elapsed,
                        avgCur
                ));
                shooterCurWriter.flush();
            }

            // ===== インテーク速度 =====
            if (intakeVelWriter != null) {
                intakeVelWriter.append(String.format(Locale.US,
                        "%d,%.2f\n",
                        elapsed,
                        intakeMotor.getVelocity()
                ));
                intakeVelWriter.flush();
            }

            // ===== インテーク電流 =====
            if (intakeCurWriter != null) {
                intakeCurWriter.append(String.format(Locale.US,
                        "%d,%.3f\n",
                        elapsed,
                        intakeMotor.getCurrent(CurrentUnit.AMPS)
                ));
                intakeCurWriter.flush();
            }

            // ===== ハブ電圧 =====
            if (hubVoltWriter != null) {
                double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
                hubVoltWriter.append(String.format(Locale.US,
                        "%d,%.3f\n",
                        elapsed,
                        voltage
                ));
                hubVoltWriter.flush();
            }

        } catch (IOException e) {
            // 書き込みエラーは無視して続行
        }
    }

    // =============================================
    // 終了
    // =============================================

    public void close() {
        closeAll();
    }

    private void closeAll() {
        closeWriter(driveVelWriter);
        closeWriter(driveCurWriter);
        closeWriter(shooterVelWriter);
        closeWriter(shooterCurWriter);
        closeWriter(intakeVelWriter);
        closeWriter(intakeCurWriter);
        closeWriter(hubVoltWriter);
    }

    private void closeWriter(FileWriter fw) {
        if (fw == null) return;
        try { fw.close(); } catch (IOException e) { /* 無視 */ }
    }
}