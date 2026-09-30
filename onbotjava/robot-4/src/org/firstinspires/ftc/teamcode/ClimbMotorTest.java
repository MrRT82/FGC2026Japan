// ============================================================================
// 最終変更: 2026-09-29 JST (UTC+09:00)
// 変更内容: クライムモーターを1.0で回すだけの確認用OpModeを新規作成。
// 今後このファイルを変更した際は、上記の日時と変更内容も更新する。
// ============================================================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// STARTからSTOPまでクライムモーターを出力1.0で回し続ける。
@TeleOp(name = "ClimbMotorTest", group = "9_TEST")
public class ClimbMotorTest extends LinearOpMode {

    static final double CLIMB_POWER = 1.0;

    @Override
    public void runOpMode() {
        // MainClassと同じ設定名"climb"のモーターを使う。
        DcMotorEx climbMotor = hardwareMap.get(DcMotorEx.class, "climb");

        telemetry.addLine("STARTでクライムモーターが1.0で回ります");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            climbMotor.setPower(CLIMB_POWER);
            telemetry.addData("power", CLIMB_POWER);
            telemetry.addData("velocity", climbMotor.getVelocity());
            telemetry.update();
        }

        climbMotor.setPower(0);
    }
}
