//ポテンショメーターのテストをするだけのコード
// 角度の換算はメインのExtendController.readAngle()に合わせる。メイン側を変えたらここも合わせる。
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="pot_angle_test", group = "zz_legacy")
public class pot_angle_test extends LinearOpMode {

    private AnalogInput pot;

    // ExtendControllerと同じ値。
    private static final double POT_FULL_SCALE_DEG = 135.0;   // 最大電圧のときの角度
    private static final double POTE_OFFSET = 33;             // ExtendController.pote_offset
    private static final double GRAVITY_OFFSET_ANGLE = 13.0;  // ExtendController.gravity_offset_angle
    private static final double POTE_ANGLE_MIN = -38;         // これ未満はセンサー異常で停止
    private static final double POTE_ANGLE_MAX = 57;          // これを超えると回復動作
    private static final double POTE_RECOVERY_MAX = 77;       // これを超えるとセンサー異常で停止

    @Override
    public void runOpMode() {
        pot = hardwareMap.get(AnalogInput.class, "potentiometer");

        telemetry.addData("Status", "Ready to measure angle");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 電圧を読み取り
            double voltage = pot.getVoltage();
            double maxVoltage = pot.getMaxVoltage();

            // 電圧 → 角度に換算(ExtendController.readAngle()と同じ)
            double angle = Double.NaN;
            if (finite(voltage) && finite(maxVoltage) && maxVoltage > 0) {
                angle = voltage / maxVoltage * POT_FULL_SCALE_DEG - POTE_OFFSET;
            }

            // メインと同じ範囲で状態を判定
            String state;
            if (!finite(angle) || angle < POTE_ANGLE_MIN || angle > POTE_RECOVERY_MAX) state = "センサー異常(メインは停止)";
            else if (angle > POTE_ANGLE_MAX) state = "回復動作の範囲(メインは低出力で下げる)";
            else state = "通常範囲";

            telemetry.addData("回った角度", "%.1f", angle);//テレメトリの数値表示 %.は少数表示、1fは小数第何位まで表示するか。0fでもいい気がする
            telemetry.addData("重力の角度(水平=0)", "%.1f", angle + GRAVITY_OFFSET_ANGLE);
            telemetry.addData("状態", state);
            telemetry.addData("電圧 / 最大電圧", "%.3f / %.3f V", voltage, maxVoltage);
            telemetry.update();
        }
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}
