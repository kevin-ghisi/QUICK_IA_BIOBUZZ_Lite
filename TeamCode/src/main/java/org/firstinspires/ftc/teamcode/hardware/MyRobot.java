package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandbase.subsystem.Drive;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.BallGate;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.FlowerCollector;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.GamePieceCoordinator;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.MagazineSensors;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.DriverFeedback;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Hood;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.commandbase.subsystem.Turret;

/** Raiz de composição: agrupa os subsistemas usados por um único OpMode. */
public final class MyRobot {
    public final Drive drive;
    public final Intake intake;
    public final Transfer transfer;
    public final Turret turret;
    public final Hood hood;
    public final Shooter shooter;
    public final FlowerCollector flowerCollector;
    public final BallGate ballGate;
    public final MagazineSensors magazine;
    public final DriverFeedback feedback;
    public final GamePieceCoordinator gamePieces;

    /** Cria todos os mecanismos uma vez, compartilhando o HardwareMap do OpMode. */
    public MyRobot(HardwareMap hardwareMap) {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);
        shooter = new Shooter(hardwareMap, hood);
        flowerCollector = new FlowerCollector(hardwareMap);
        ballGate = new BallGate(hardwareMap);
        magazine = new MagazineSensors(hardwareMap);
        feedback = new DriverFeedback(hardwareMap);
        gamePieces = new GamePieceCoordinator(intake, shooter, flowerCollector,
                ballGate, turret, magazine, transfer);
    }

    /** Inicializa cada subsistema no começo do OpMode e deixa mecanismos móveis seguros. */
    public void initializeForOpMode() {
        drive.initialize();
        intake.initialize();
        transfer.initialize();
        turret.initialize();
        hood.initialize();
        shooter.initialize();
        flowerCollector.initialize();
        ballGate.initialize();
        magazine.initialize();
        feedback.initialize();
        gamePieces.initialize();
    }
}
