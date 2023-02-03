package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drivetrain;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;

public class VisionCommandBuilder {

    private PhotonCamera camera;
    private Drivetrain drivetrain;
    private Supplier<Pose2d> poseProvider;
    private int offset = 0;

    public VisionCommandBuilder(PhotonCamera camera, Drivetrain drivetrain, Supplier<Pose2d> poseProvider)
    {
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;  
    }

    public Command increaseOffset() {
        return new InstantCommand(() -> offset++);
    }

    public Command decreaseOffset() {
        return new InstantCommand(() -> offset--);
    }

    public Command alignToTag() {
        return new AlignToTag(camera, drivetrain, poseProvider, offset);
    }
}
