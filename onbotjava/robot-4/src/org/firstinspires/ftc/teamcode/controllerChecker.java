package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="controllerChecker", group = "Diag")
public class controllerChecker extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "準備完了");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== Gamepad1 =====
            telemetry.addData("=== Gamepad1 ===", "");

            // スティック
            telemetry.addData("左スティックX",  "%.3f", gamepad1.left_stick_x);
            telemetry.addData("左スティックY",  "%.3f", gamepad1.left_stick_y);
            telemetry.addData("右スティックX",  "%.3f", gamepad1.right_stick_x);
            telemetry.addData("右スティックY",  "%.3f", gamepad1.right_stick_y);
            telemetry.addData("左スティックボタン", gamepad1.left_stick_button);
            telemetry.addData("右スティックボタン", gamepad1.right_stick_button);

            // トリガー
            telemetry.addData("左トリガー",  "%.3f", gamepad1.left_trigger);
            telemetry.addData("右トリガー",  "%.3f", gamepad1.right_trigger);

            // バンパー
            telemetry.addData("左バンパー",  gamepad1.left_bumper);
            telemetry.addData("右バンパー",  gamepad1.right_bumper);

            // ボタン
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);

            // dpad
            telemetry.addData("dpad_up",    gamepad1.dpad_up);
            telemetry.addData("dpad_down",  gamepad1.dpad_down);
            telemetry.addData("dpad_left",  gamepad1.dpad_left);
            telemetry.addData("dpad_right", gamepad1.dpad_right);

            // その他
            telemetry.addData("start",  gamepad1.start);
            telemetry.addData("back",   gamepad1.back);

            // ===== Gamepad2 =====
            telemetry.addData("=== Gamepad2 ===", "");

            // スティック
            telemetry.addData("左スティックX",  "%.3f", gamepad2.left_stick_x);
            telemetry.addData("左スティックY",  "%.3f", gamepad2.left_stick_y);
            telemetry.addData("右スティックX",  "%.3f", gamepad2.right_stick_x);
            telemetry.addData("右スティックY",  "%.3f", gamepad2.right_stick_y);
            telemetry.addData("左スティックボタン", gamepad2.left_stick_button);
            telemetry.addData("右スティックボタン", gamepad2.right_stick_button);

            // トリガー
            telemetry.addData("左トリガー",  "%.3f", gamepad2.left_trigger);
            telemetry.addData("右トリガー",  "%.3f", gamepad2.right_trigger);

            // バンパー
            telemetry.addData("左バンパー",  gamepad2.left_bumper);
            telemetry.addData("右バンパー",  gamepad2.right_bumper);

            // ボタン
            telemetry.addData("A", gamepad2.a);
            telemetry.addData("B", gamepad2.b);
            telemetry.addData("X", gamepad2.x);
            telemetry.addData("Y", gamepad2.y);

            // dpad
            telemetry.addData("dpad_up",    gamepad2.dpad_up);
            telemetry.addData("dpad_down",  gamepad2.dpad_down);
            telemetry.addData("dpad_left",  gamepad2.dpad_left);
            telemetry.addData("dpad_right", gamepad2.dpad_right);

            // その他
            telemetry.addData("start",  gamepad2.start);
            telemetry.addData("back",   gamepad2.back);

            telemetry.update();
        }
    }
}