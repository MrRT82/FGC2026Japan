//ポテンショメーターのテストをするだけのコード
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="pot_angle_test", group = "zz_legacy")
public class pot_angle_test extends LinearOpMode {

    private AnalogInput pot;

    private static final double POT_SUPPLY = 3.3;   // ポテンショメータ供給電圧（REV Hubは3.3V）絶対固定
    private static final double POT_MAX_DEG = 270.0; // 可動域(poteを270度回したときに認識される角度。変える意味はよく分からない)

    @Override
    public void runOpMode() {
        pot = hardwareMap.get(AnalogInput.class, "potentiometer");

        telemetry.addData("Status", "Ready to measure angle");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 電圧を読み取り
            double voltage = pot.getVoltage();

            // 電圧 → 角度に換算
            double angle = (voltage / POT_SUPPLY) * POT_MAX_DEG / 2 - 21.6;

            // Driver Hub には角度のみ表示
            telemetry.addData("回った角度", "%.1f", angle);//テレメトリの数値表示 %.は少数表示、1fは小数第何位まで表示するか。0fでもいい気がする
            telemetry.update();
        }
    }
}
