// package frc.robot.subsystems.swervedrive;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

// public class AutoIntakeHP extends Command {
//     private SwerveCommands swerveCommands;
//     private SwerveSubsystem swerve;
//     private Pose2d target;

//     public AutoIntakeHP(SwerveSubsystem swerve, SwerveCommands swerveCommands) {
//         this.swerve = swerve;
//         this.swerveCommands = swerveCommands;
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {
//         // new AutoAlignNear();

//     }

//     @Override
//     public void isFinished() {

//     }
// }
// /*
//  * public Pose2d autoIntakePose(){
//  * if (PoseTransformUtils.isRedAlliance()) {
//  * if (Vision.getAprilTagPose(1).getTranslation().getDistance(swerve.getPose().
//  * getTranslation()) >= Vision
//  * .getAprilTagPose(2).getTranslation().getDistance(swerve.getPose().
//  * getTranslation())) {
//  * return new Pose2d()
//  * } else {
//  * pose = new
//  * }
//  * } else {
//  * if (Vision.getAprilTagPose(13).getTranslation().getDistance(swerve.getPose().
//  * getTranslation()) >= Vision
//  * .getAprilTagPose(12).getTranslation().getDistance(swerve.getPose().
//  * getTranslation())) {
//  * pose =
//  * } else {
//  * pose =
//  * }
//  * }
//  * 
//  * return pose;
//  * }
//  * 
//  * public Command autoIntakeCoral() {
//  * boolean isRed = PoseTransformUtils.isRedAlliance();
//  * Commands.either(Commands.print("command 1"), Commands.print("Command 2"), ()
//  * -> PoseTransformUtils.isRedAlliance());
//  * Optional<Boolean> isRight = Optional.of(right);
//  * 
//  * var command = Commands.sequence(
//  * new AutoAlignFar(autoIntakePose()),
//  * new AutoAlignNear());
//  * return command;
//  * }
//  */