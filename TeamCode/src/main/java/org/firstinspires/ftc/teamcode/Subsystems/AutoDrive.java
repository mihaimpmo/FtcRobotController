//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//
//
///**
// * Controleaza miscarile autonome ale robotului folosind odometria Pinpoint.
// *
// * Ideea de baza: in loc sa ne bazam pe pozitia actuala a robotului ca referinta,
// * folosim o pozitie "expectedPose" adica unde TEORETIC ar trebui sa fie robotul
// * dupa toate miscarile anterioare. Asta previne acumularea driftului de la o miscare la alta.
// *
// * Semnul negativ pe drive.drive(-power, ...) vine din faptul ca -power = forward
// * in implementarea curenta a swerve-ului (CCW-positive kinematics).
// */
//@Configurable
//public class AutoDrive {
//
//    // Puterea maxima pentru drive/turn, valori intre 0 si 1
//    public static double DRIVE_POWER = 1;
//    public static double TURN_POWER = 1;
//
//    // Cat de puternic corectam heading-ul in timpul mersului.
//    // Trebuie sa fie FOARTE mic (0.03) ca sa nu faca S-uri pe teren.
//    // Fara deadband — presiune mica si constanta e mai stabila decat corecturi on/off.
//    public static double HEADING_CORRECTION_P = 0.03;
//
//    // Cat de puternic corectam driftul pe axa perpendiculara pe directia de mers.
//    // Exemplu: daca mergem forward si robotul deviaza pe Y, asta il impinge inapoi.
//    public static double CROSS_AXIS_P = 0.05;
//
//    // Cand robotul e la mai putin de DECEL_DISTANCE inch de target, incepe sa incetineasca.
//    // Rampa liniara: de la maxPower la MIN_POWER pe masura ce se apropie.
//    public static double DECEL_DISTANCE = 8.0;    // inches
//    public static double DECEL_ANGLE = 15.0;       // degrees, la fel dar pt turnTo
//    public static double MIN_POWER = 0.15;         // puterea minima ca sa nu se opreasca prematur
//
//    // Dupa ce opreste motoarele, asteapta SETTLE_MS ca robotul sa se stabilizeze fizic
//    public static long SETTLE_MS = 200;
//
//    private final SwerveDrive drive;
//    private final GoBildaPinpointDriver pinpoint;
//    private final LinearOpMode opMode;
//
//    // Pozitia teoretica  undje "ar trebui" sa fie robotul.
//    // Se initializeaza la (0,0,0) dupa reset-ul IMU si se actualizeaza dupa fiecare miscare.
//    private Pose2D expectedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
//
//    public AutoDrive(SwerveDrive drive, GoBildaPinpointDriver pinpoint, LinearOpMode opMode) {
//        this.drive = drive;
//        this.pinpoint = pinpoint;
//        this.opMode = opMode;
//        // Citim pozitia actuala ca baseline  dupa resetul din Auto.java, ar trebui sa fie (0,0,0)
//        this.expectedPose = readPose();
//    }
//
//    /** Citeste pozitia curenta de la Pinpoint. Apeleaza update() ca sa ia cele mai recente date. */
//    private Pose2D readPose() {
//        pinpoint.update();
//        return pinpoint.getPosition();
//    }
//
//    /**
//     * Afiseaza cat de departe e robotul de target dupa o miscare (drift report).
//     * Dupa afisare, seteaza expectedPose la TARGET (nu la pozitia reala)
//     * asta e ideea :)) urmatoarea miscare va pleca de la unde TREBUIA sa ajunga,
//     * nu de la unde a ajuns de fapt. Pinpoint-ul va vedea diferenta si va compensa.
//     */
//    private void logDrift(double targetX, double targetY, double targetHeading) {
//        Pose2D currentPose = readPose();
//        double errorX = currentPose.getX(DistanceUnit.INCH) - targetX;
//        double errorY = currentPose.getY(DistanceUnit.INCH) - targetY;
//        double errorHeading = normalizeAngle(currentPose.getHeading(AngleUnit.DEGREES) - targetHeading);
//
//        opMode.telemetry.addLine("--- DRIFT REPORT ---");
//        opMode.telemetry.addData("X Error", "%.2f in", errorX);
//        opMode.telemetry.addData("Y Error", "%.2f in", errorY);
//        opMode.telemetry.addData("H Error", "%.1f°", errorHeading);
//        opMode.telemetry.update();
//
//        // Baseline-ul devine targetul teoretic, nu pozitia reala
//        expectedPose = new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES, targetHeading);
//    }
//
//    /**
//     * Calculeaza puterea cu rampa de decelerare.
//     * Cand distanta ramasa > decelZone: putere maxima.
//     * Cand distanta ramasa < decelZone: putere scade liniar pana la MIN_POWER.
//     * Semnul returnat reflecta directia (pozitiv/negativ).
//     */
//    private double rampedPower(double maxPower, double remaining, double decelZone) {
//        double absRemaining = Math.abs(remaining);
//        double power;
//        if (absRemaining > decelZone) {
//            power = maxPower;
//        } else {
//            power = MIN_POWER + (maxPower - MIN_POWER) * (absRemaining / decelZone);
//        }
//        return power * Math.signum(remaining);
//    }
//
//    /**
//     * Merge inainte/inapoi pe axa X cu corectii active pe Y si heading.
//     * Valori pozitive = inainte, negative = inapoi.
//     *
//     * Corectii in timpul miscarii:
//     *   - strafeCorrection: daca robotul deviaza pe Y, il impinge inapoi (robot-centric nudge)
//     *   - rotCorrection: daca heading-ul se schimba, il corecteaza usor
//     *
//     * drive.drive(-power, ...) cu minus pt ca -fwd = forward in swerve-ul nostru.
//     */
//    public void forward(double inches) {
//        // Targetul se calculeaza relativ la expectedPose, nu la pozitia actuala
//        double targetX = expectedPose.getX(DistanceUnit.INCH) + inches;
//        double targetY = expectedPose.getY(DistanceUnit.INCH);
//        double targetHeading = expectedPose.getHeading(AngleUnit.DEGREES);
//
//        while (opMode.opModeIsActive()) {
//            Pose2D pose = readPose();
//            double currentX = pose.getX(DistanceUnit.INCH);
//            double currentY = pose.getY(DistanceUnit.INCH);
//            double currentHeading = pose.getHeading(AngleUnit.DEGREES);
//
//            double remaining = targetX - currentX;
//            // Am ajuns la target (sau l-am depasit)
//            if (inches >= 0 && remaining <= 0) break;
//            if (inches < 0 && remaining >= 0) break;
//
//            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE);
//
//            // Corectie laterala: cat de mult a deviat pe Y fata de linia pe care trebuia sa mearga
//            double yDrift = currentY - targetY;
//            double strafeCorrection = CROSS_AXIS_P * yDrift;
//
//            // Corectie heading: diferenta fata de unghiul pe care trebuia sa-l mentina
//            double headingError = normalizeAngle(targetHeading - currentHeading);
//            double rotCorrection = HEADING_CORRECTION_P * headingError;
//
//            // -power = forward, strafeCorrection = lateral nudge, -rotCorrection = heading nudge
//            // Semnele negative vin din conventiile CCW-positive ale kinematics-ului
//            drive.drive(-power, strafeCorrection, -rotCorrection);
//            drive.update();
//
//            opMode.telemetry.addData("Forward", "%.1f / %.1f in", currentX - expectedPose.getX(DistanceUnit.INCH), inches);
//            opMode.telemetry.addData("Drift", "Y: %.2f, H: %.1f", yDrift, headingError);
//            opMode.telemetry.update();
//        }
//        brakeAndSettle();
//        logDrift(targetX, targetY, targetHeading);
//    }
//
//    /**
//     * Merge lateral (strafe) pe axa Y cu corectii active pe X si heading.
//     * Valori pozitive = stanga, negative = dreapta.
//     *
//     * Acelasi principiu ca forward(), dar pe axa perpendiculara:
//     *   - fwdCorrection: daca robotul deviaza pe X, il impinge inapoi
//     *   - rotCorrection: corectie heading
//     */
//    public void strafe(double inches) {
//        double targetX = expectedPose.getX(DistanceUnit.INCH);
//        double targetY = expectedPose.getY(DistanceUnit.INCH) + inches;
//        double targetHeading = expectedPose.getHeading(AngleUnit.DEGREES);
//
//        while (opMode.opModeIsActive()) {
//            Pose2D pose = readPose();
//            double currentX = pose.getX(DistanceUnit.INCH);
//            double currentY = pose.getY(DistanceUnit.INCH);
//            double currentHeading = pose.getHeading(AngleUnit.DEGREES);
//
//            double remaining = targetY - currentY;
//            if (inches >= 0 && remaining <= 0) break;
//            if (inches < 0 && remaining >= 0) break;
//
//            double power = rampedPower(DRIVE_POWER, remaining, DECEL_DISTANCE);
//
//            // Corectie longitudinala: cat de mult a deviat pe X
//            double xDrift = currentX - targetX;
//            double fwdCorrection = CROSS_AXIS_P * xDrift;
//
//            double headingError = normalizeAngle(targetHeading - currentHeading);
//            double rotCorrection = HEADING_CORRECTION_P * headingError;
//
//            // fwdCorrection = longitudinal nudge, -power = strafe, -rotCorrection = heading nudge
//            drive.drive(fwdCorrection, -power, -rotCorrection);
//            drive.update();
//
//            opMode.telemetry.addData("Strafe", "%.1f / %.1f in", currentY - expectedPose.getY(DistanceUnit.INCH), inches);
//            opMode.telemetry.addData("Drift", "X: %.2f, H: %.1f", xDrift, headingError);
//            opMode.telemetry.update();
//        }
//        brakeAndSettle();
//        logDrift(targetX, targetY, targetHeading);
//    }
//
//    /**
//     * Se roteste pana la un heading absolut (in grade).
//     * targetHeading e heading-ul final dorit, NU o rotatie relativa.
//     * Deadband de 3 grade cand eroarea e sub 3°, consideram ca am ajuns.
//     *
//     * Puterea se inverseaza (-power) din cauza conventiei CCW-positive:
//     * o eroare pozitiva inseamna ca trebuie sa mergem CCW, dar kinematics-ul
//     * interpreteaza rot pozitiv ca CCW, deci trebuie inversat semnul.
//     */
//    public void turnTo(double targetHeading) {
//        double targetX = expectedPose.getX(DistanceUnit.INCH);
//        double targetY = expectedPose.getY(DistanceUnit.INCH);
//
//        while (opMode.opModeIsActive()) {
//            Pose2D pose = readPose();
//            double heading = pose.getHeading(AngleUnit.DEGREES);
//            double error = normalizeAngle(targetHeading - heading);
//
//            if (Math.abs(error) < 3.0) break;
//
//            double power = rampedPower(TURN_POWER, error, DECEL_ANGLE);
//
//            // Doar rotatie, fara translatie
//            drive.drive(0, 0, -power);
//            drive.update();
//
//            opMode.telemetry.addData("Turn", "%.1f° → %.1f°", heading, targetHeading);
//            opMode.telemetry.update();
//        }
//        brakeAndSettle();
//        logDrift(targetX, targetY, targetHeading);
//    }
//
//    /** Opreste motoarele si asteapta SETTLE_MS ca inertia sa se disipeze. */
//    private void brakeAndSettle() {
//        drive.drive(0, 0, 0);
//        drive.update();
//        opMode.sleep(SETTLE_MS);
//    }
//
//    /** Opreste motoarele imediat, fara settle. Folosit la cleanup la finalul auto-ului. */
//    public void stop() {
//        drive.drive(0, 0, 0);
//        drive.update();
//    }
//
//    /** Normalizeaza un unghi in grade la intervalul [-180, 180]. */
//    private static double normalizeAngle(double degrees) {
//        degrees = degrees % 360;
//        if (degrees > 180) degrees -= 360;
//        if (degrees < -180) degrees += 360;
//        return degrees;
//    }
//}
