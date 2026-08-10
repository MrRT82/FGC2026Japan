package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainClass_Blue", group="Test")
public class MainClass_Blue extends LinearOpMode {

    private RobotHardware hardware;
    private DriveController driveController;
    private ShooterController shooterController;
    private ClimbController climbController;
    private IntakeController intakeController;
    private MotorLogger motorLogger;
    private ExtendController extendController;
    private FeedController feedController;
    private ImuController imuController;

    boolean prevUp_2 = false;
    boolean prevDown_2 = false;
    boolean prevRight_2 = false;
    boolean prevLeft_2 = false;
    boolean prevLeftBump = false;
    boolean prevA = false;
    double prevLeftT;
    boolean isStart = false;
    boolean isShootMode = false;
    boolean wasShoot = false;
    boolean isClimbMode = false;
    int climbEnum = 0;
    int climbTime = 120;
    boolean timerRunning = false;
    boolean isReverse = false;
    double shootSpeed = 1500;
    int i = 0;
    int prevLoop = 3;
    double climbTarget = 16; //Blue 16,Red -16
    double shootTarget = -90; // Blue -90,Red 90
    double shootTargetB = -30; //Blue -30,Red 60a


    @Override
    public void runOpMode() {

        hardware = new RobotHardware();
        hardware.init(hardwareMap);
        motorLogger = new MotorLogger(hardware, hardwareMap);
        motorLogger.init();

        driveController = new DriveController(hardware);
        shooterController = new ShooterController(hardware);
        climbController = new ClimbController(hardware);
        intakeController = new IntakeController(hardware);
        feedController = new FeedController(hardware);
        extendController = new ExtendController(hardware);
        imuController = new ImuController(hardware);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime climbTimer = new ElapsedTime();
        ElapsedTime allTimer = new ElapsedTime();

        telemetry.addData("Status", "準備完了");
        telemetry.update();

        waitForStart();

        imuController.reset();

        while (opModeIsActive()) {
            double forwardRaw = driveController.applyDeadzone(-gamepad1.left_stick_y);
            double turnRaw = driveController.applyDeadzone(gamepad1.right_stick_x);

            double Angle = imuController.getHeading();

            driveController.driveNormal(forwardRaw, turnRaw);

            extendController.extend_loop();
            shooterController.shooter_loop();

            if (isStart) {
                double currentVelocity = Math.abs(intakeController.Intake.getVelocity());
                if (!isReverse && !isShootMode && currentVelocity < 100) {
                    if (!timerRunning) {
                        timer.reset();
                        timerRunning = true;
                    } else if (timer.milliseconds() > 300) {
                        isReverse = true;
                        timerRunning = false;
                        timer.reset();
                    }
                } else if (!isReverse) {
                    timerRunning = false;
                }
                if (isReverse) {
                    if (timer.milliseconds() < 300) {
                        intakeController.Intake_Reverse();
                    } else {
                        isReverse = false;
                    }
                } else {

                    if (gamepad1.b) {
                        intakeController.Intake_Reverse();
                        feedController.Feed_turn_left();
                        extendController.extendUp();
                    } else if (!isShootMode) {
                        intakeController.Intake_Forward();
                        feedController.Feed_turn_right();
                    }
                }

                if (gamepad1.left_bumper) {// && !prevLeftBump エッジ判定はいらない気がしている　だるい
                    extendController.extendDown();
                }

                if (gamepad1.right_trigger > 0.1 && !isShootMode && !isClimbMode) {
                    isShootMode = true;
                    imuController.setTarget(shootTarget);
                    shooterController.setSpeed(shootSpeed);
                }else if(gamepad1.a && !prevA && !isShootMode) {
                    isShootMode = true;
                    imuController.setTarget(shootTargetB);
                    shooterController.setSpeed(shootSpeed);
                }

                if(!isShootMode && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
                    isClimbMode = true;
                }
                if(isClimbMode && climbTarget + 90 >= Angle && Angle >= climbTarget - 90){
                    if(imuController.getTarget() != climbTarget){
                        imuController.setTarget(climbTarget);
                    }
                    imuController.update();
                }else if(isClimbMode){
                    if(imuController.getTarget() != climbTarget - 180 || imuController.getTarget() != climbTarget + 180){
                        if(climbTarget > 0){
                            imuController.setTarget(climbTarget - 180);
                        }else{
                            imuController.setTarget(climbTarget + 180);
                        }
                    }
                    imuController.update();
                }

                if (isShootMode) {
                    imuController.update();
                    if (gamepad1.right_bumper) {
                        feedController.Feed_turn_left();
                        extendController.extendUp();
                        wasShoot = true;
                    } else if (!gamepad1.right_bumper && wasShoot) {
                        shooterController.setSpeed(0);
                        wasShoot = false;
                        isShootMode = false;
                    }
                }
                if(gamepad1.left_stick_button || gamepad1.right_stick_button){
                    isShootMode = false;
                    isClimbMode = false;
                    shooterController.setSpeed(0);
                }

                if (gamepad2.left_trigger > 0.05 && climbTimer.milliseconds() > 300) {
                    climbEnum += 1;
                    climbTimer.reset();
                    if (climbEnum > 5) {
                        climbEnum = 2;
                    }
                }
                if(gamepad1.left_trigger > 0.05 && climbEnum != 0 && climbTimer.milliseconds() > 300){
                    climbEnum += 1;
                    climbTimer.reset();
                    if(climbEnum > 5){
                        climbEnum = 2;
                    }
                }
                if(allTimer.seconds() > climbTime && climbEnum == 0){
                    climbEnum = 1;
                }
                switch (climbEnum) {
                    case 0:
                        climbController.climbStop();
                        break;
                    case 1:
                        climbController.climbOpen();
                        break;
                    case 2:
                        if(!climbController.isBottomLimit()){
                            climbController.climbDown();
                        } else {
                            climbController.climbStop();
                        }
                        break;
                    case 3:
                        climbController.climbStop();
                        break;
                    case 4:
                        climbController.climbUp();
                        break;
                    case 5:
                        climbController.climbStop();
                        break;
                }

                int cS = (int) (shooterController.leftShooter.getVelocity() + shooterController.rightShooter.getVelocity()) / 2;

                telemetry.addData("目標角度", extendController.targetAngle);
                telemetry.addData("現在角度", extendController.current);
                telemetry.addData("現在出力", extendController.output);
                telemetry.addData("現在速度", cS);
                telemetry.addData("詰まり検知タイマー", timerRunning ? timer.milliseconds() : 0);
                telemetry.addData("自動逆転中", isReverse);
                telemetry.addData("シューター対目標速度誤差",Math.abs((shooterController.leftShooter.getVelocity() + shooterController.rightShooter.getVelocity()) / 2 - shootSpeed));
                telemetry.addData("シューター速度左",shooterController.leftShooter.getVelocity());
                telemetry.addData("シューター速度右",shooterController.rightShooter.getVelocity());
                telemetry.addData("クライムタイマー",climbTimer.milliseconds());
                telemetry.addData("ポテンショメーター",extendController.current);
                telemetry.addData("展開詰まり", extendController.isStuck ? "詰まり" : "正常");
                telemetry.addData("現在角度",imuController.getHeading());
                telemetry.addData("目標角度",imuController.getTarget());
                telemetry.update();

                motorLogger.update();

                i ++;
                if(i >= prevLoop){
                    i = 0;
                    prevDown_2 = gamepad2.dpad_down;
                    prevUp_2 = gamepad2.dpad_up;
                    prevRight_2 = gamepad2.dpad_right;
                    prevLeft_2 = gamepad2.dpad_left;
                    prevLeftBump = gamepad1.left_bumper;
                    prevA = gamepad1.a;

                }

            } else {
                telemetry.addData("展開出力",extendController.output);
                telemetry.update();
                allTimer.reset();
                if (Math.abs(gamepad1.left_stick_y) > 0.05) {
                    isStart = true;
                }
            }
        }
        motorLogger.close();
    }
}