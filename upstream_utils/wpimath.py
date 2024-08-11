#!/usr/bin/env python3

from pathlib import Path
from typing import List
import shutil
import os
import re

from upstream_utils import Lib


def copy_file(wpilib_path: Path, ftclib_path: Path):
    ftclib_path.parent.mkdir(parents=True, exist_ok=True)

    # Copy the file
    shutil.copy(wpilib_path, ftclib_path)

    try:
        contents = ftclib_path.read_text(encoding="utf-8")
    except:
        print("Failed to convert ", wpilib_path)
        return

    # Remap packages
    contents = contents.replace("edu.wpi.first.math", "com.arcrobotics.ftclib")
    contents = contents.replace("edu.wpi.first.wpilibj", "com.arcrobotics.ftclib")
    contents = contents.replace("edu.wpi.first.wpilibj2", "com.arcrobotics.ftclib")
    contents = contents.replace("edu.wpi.first.wpiutil", "com.arcrobotics.ftclib.util")
    contents = contents.replace("edu.wpi.first.util", "com.arcrobotics.ftclib.util")

    # Remap certain classes that get lost in the shuffle above
    contents = contents.replace(
        "edu.wpi.first.util.ErrorMessages", "com.arcrobotics.ftclib.util.ErrorMessages"
    )
    contents = contents.replace(
        "com.arcrobotics.ftclib.MathUtil", "com.arcrobotics.ftclib.util.MathUtil"
    )

    # Remap things that are put into special packages
    contents = contents.replace(
        "package com.arcrobotics.ftclib.controller;",
        "package com.arcrobotics.ftclib.controller.wpilibcontroller;",
    )
    contents = contents.replace(
        "package com.arcrobotics.ftclib.kinematics;",
        "package com.arcrobotics.ftclib.kinematics.wpilibkinematics;",
    )
    contents = contents.replace(
        "com.arcrobotics.ftclib2.command", "com.arcrobotics.ftclib.command"
    )

    contents = contents.replace(
        "import com.arcrobotics.ftclib.controller.SimpleMotorFeedforward;",
        "import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.controller.ProfiledPIDController;",
        "import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.controller.RamseteController;",
        "import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;",
    )

    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.ChassisSpeeds;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.DifferentialDriveKinematics;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.MecanumDriveKinematics;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.MecanumDriveWheelSpeeds;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.SwerveDriveKinematics;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.SwerveModuleState;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;",
    )
    contents = contents.replace(
        "import com.arcrobotics.ftclib.kinematics.DifferentialDriveWheelSpeeds;",
        "import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;",
    )

    # We will trim any special info off the deprecated tags
    contents = re.sub("@Deprecated.*", "@Deprecated", contents)

    # Delete MathShared
    contents = re.sub("MathSharedStore.reportUsage.*\n", "", contents)
    contents = re.sub("(MathSharedStore.reportError.*\n)", r"//\1", contents)
    contents = re.sub("import com.arcrobotics.ftclib.MathSharedStore;\n", "", contents)
    contents = re.sub("import com.arcrobotics.ftclib.MathUsageId;\n", "", contents)

    # Replace json
    contents = re.sub(r"@JsonIgnoreProperties\(.*?\)", "", contents)
    contents = re.sub(r"@JsonAutoDetect\(.*?\)", "", contents)
    contents = re.sub("import com.fasterxml.*", "", contents)
    contents = re.sub("@JsonCreator", "", contents)
    contents = re.sub(r"@JsonProperty(\r\n|\r|\n)", "", contents)
    contents = re.sub(r"@JsonProperty\(.*?\)(\r\n|\r|\n)", "", contents)
    contents = re.sub(r"@JsonProperty\(.*?\)", "", contents)

    ftclib_path.write_text(contents, encoding="utf-8")


def copy_geometry(wpilib_root, ftclib_root):
    files = [
        "Pose2d.java",
        "Rotation2d.java",
        "Transform2d.java",
        "Translation2d.java",
        "Twist2d.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/main/java/edu/wpi/first/math/geometry" / f,
            ftclib_root / "core/src/main/java/com/arcrobotics/ftclib/geometry" / f,
        )

    files = [
        "Pose2dTest.java",
        "Rotation2dTest.java",
        "Transform2dTest.java",
        "Translation2dTest.java",
        "Twist2dTest.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/test/java/edu/wpi/first/math/geometry" / f,
            ftclib_root / "core/src/test/java/com/arcrobotics/ftclib/geometry" / f,
        )


def copy_trajectory(wpilib_root, ftclib_root):
    files = [
        "Trajectory.java",
        "TrajectoryConfig.java",
        "TrajectoryGenerator.java",
        "TrajectoryParameterizer.java",
        "TrapezoidProfile.java",
        "constraint/CentripetalAccelerationConstraint.java",
        "constraint/DifferentialDriveKinematicsConstraint.java",
        "constraint/DifferentialDriveVoltageConstraint.java",
        "constraint/MecanumDriveKinematicsConstraint.java",
        "constraint/SwerveDriveKinematicsConstraint.java",
        "constraint/TrajectoryConstraint.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/main/java/edu/wpi/first/math/trajectory" / f,
            ftclib_root / "core/src/main/java/com/arcrobotics/ftclib/trajectory" / f,
        )

    files = [
        "CentripetalAccelerationConstraintTest.java",
        "DifferentialDriveKinematicsConstraintTest.java",
        "DifferentialDriveVoltageConstraintTest.java",
        "TrajectoryConcatenateTest.java",
        "TrajectoryGeneratorTest.java",
        "TrajectoryTransformTest.java",
        "TrapezoidProfileTest.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/test/java/edu/wpi/first/math/trajectory" / f,
            ftclib_root / "core/src/test/java/com/arcrobotics/ftclib/trajectory" / f,
        )


def copy_spline(wpilib_root, ftclib_root):
    files = [
        "CubicHermiteSpline.java",
        "PoseWithCurvature.java",
        "QuinticHermiteSpline.java",
        "Spline.java",
        "SplineHelper.java",
        "SplineParameterizer.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/main/java/edu/wpi/first/math/spline" / f,
            ftclib_root / "core/src/main/java/com/arcrobotics/ftclib/spline" / f,
        )

    files = [
        "CubicHermiteSplineTest.java",
        "QuinticHermiteSplineTest.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/test/java/edu/wpi/first/math/spline" / f,
            ftclib_root / "core/src/test/java/com/arcrobotics/ftclib/spline" / f,
        )


def copy_kinematics(wpilib_root, ftclib_root):
    files = [
        "ChassisSpeeds.java",
        "DifferentialDriveKinematics.java",
        "DifferentialDriveOdometry.java",
        "DifferentialDriveWheelSpeeds.java",
        "DifferentialDriveWheelPositions.java",
        "Kinematics.java",
        "MecanumDriveKinematics.java",
        "MecanumDriveMotorVoltages.java",
        "MecanumDriveOdometry.java",
        "MecanumDriveWheelSpeeds.java",
        "MecanumDriveWheelPositions.java",
        "Odometry.java",
        "SwerveDriveKinematics.java",
        "SwerveDriveOdometry.java",
        "SwerveDriveWheelPositions.java",
        "SwerveModulePosition.java",
        "SwerveModuleState.java",
        "WheelPositions.java",
    ]

    for f in files:
        ftc_file = (
            ftclib_root
            / "core/src/main/java/com/arcrobotics/ftclib/kinematics/wpilibkinematics/"
            / f
        )
        copy_file(
            wpilib_root / "wpimath/src/main/java/edu/wpi/first/math/kinematics" / f,
            ftc_file,
        )

    files = [
        "ChassisSpeedsTest.java",
        "DifferentialDriveKinematicsTest.java",
        "DifferentialDriveOdometryTest.java",
        "MecanumDriveKinematicsTest.java",
        "MecanumDriveOdometryTest.java",
        "SwerveDriveKinematicsTest.java",
        "SwerveDriveOdometryTest.java",
        "SwerveModuleStateTest.java",
    ]

    for f in files:
        ftc_file = (
            ftclib_root
            / "core/src/test/java/com/arcrobotics/ftclib/kinematics/wpilibkinematics/"
            / f
        )
        copy_file(
            wpilib_root / "wpimath/src/test/java/edu/wpi/first/math/kinematics" / f,
            ftc_file,
        )


def copy_controller(wpilib_root, ftclib_root):
    files = [
        "ArmFeedforward.java",
        "ElevatorFeedforward.java",
        "HolonomicDriveController.java",
        "ProfiledPIDController.java",
        "RamseteController.java",
        "SimpleMotorFeedforward.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/main/java/edu/wpi/first/math/controller" / f,
            ftclib_root
            / "core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller"
            / f,
        )

    files = [
        "HolonomicDriveControllerTest.java",
        "ProfiledPIDControllerTest.java",
        "RamseteControllerTest.java",
        # "SimpleMotorFeedforwardTest.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/test/java/edu/wpi/first/math/controller" / f,
            ftclib_root
            / "core/src/test/java/com/arcrobotics/ftclib/controller/wpilibcontroller"
            / f,
        )


def copy_interpolation(wpilib_root, ftclib_root):
    files = [
        "Interpolatable.java",
        "InterpolatingDoubleTreeMap.java",
        "InterpolatingTreeMap.java",
        "Interpolator.java",
        "InverseInterpolator.java",
        "TimeInterpolatableBuffer.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/main/java/edu/wpi/first/math/interpolation" / f,
            ftclib_root / "core/src/main/java/com/arcrobotics/ftclib/interpolation" / f,
        )

    files = [
        "InterpolatingDoubleTreeMapTest.java",
        "InterpolatingTreeMapTest.java",
        "TimeInterpolatableBufferTest.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpimath/src/test/java/edu/wpi/first/math/interpolation" / f,
            ftclib_root / "core/src/test/java/com/arcrobotics/ftclib/interpolation" / f,
        )


def copy_wpimath_utils(wpilib_root, ftclib_root):
    files = [
        (
            "wpimath/src/main/java/edu/wpi/first/math/MathUtil.java",
            "core/src/main/java/com/arcrobotics/ftclib/util/MathUtil.java",
        ),
        (
            "wpimath/src/main/java/edu/wpi/first/math/util/Units.java",
            "core/src/main/java/com/arcrobotics/ftclib/util/Units.java",
        ),
        # Tests
        (
            "wpimath/src/test/java/edu/wpi/first/math/MathUtilTest.java",
            "core/src/test/java/com/arcrobotics/ftclib/util/MathUtilTest.java",
        ),
        (
            "wpimath/src/test/java/edu/wpi/first/math/util/UnitsTest.java",
            "core/src/test/java/com/arcrobotics/ftclib/util/UnitsTest.java",
        ),
    ]

    for wpi_path, ftc_path in files:
        copy_file(
            wpilib_root / wpi_path,
            ftclib_root / ftc_path,
        )

    # Hand cleanup mathutil
    mathutil = (
        ftclib_root / "core/src/main/java/com/arcrobotics/ftclib/util/MathUtil.java"
    )
    contents = mathutil.read_text(encoding="utf-8")
    contents = contents.replace(
        "package com.arcrobotics.ftclib;", "package com.arcrobotics.ftclib.util;"
    )
    mathutil.write_text(contents, encoding="utf-8")

    mathutil = (
        ftclib_root / "core/src/test/java/com/arcrobotics/ftclib/util/MathUtilTest.java"
    )
    contents = mathutil.read_text(encoding="utf-8")
    contents = contents.replace(
        "package com.arcrobotics.ftclib;", "package com.arcrobotics.ftclib.util;"
    )
    mathutil.write_text(contents, encoding="utf-8")


def copy_wpiutil(wpilib_root, ftclib_root):
    files = [
        "ErrorMessages.java",
    ]

    for f in files:
        ftc_file = ftclib_root / "core/src/main/java/com/arcrobotics/ftclib/util" / f
        copy_file(
            wpilib_root / "wpiutil/src/main/java/edu/wpi/first/util" / f,
            ftc_file,
        )

    files = [
        "ErrorMessagesTest.java",
    ]

    for f in files:
        copy_file(
            wpilib_root / "wpiutil/src/test/java/edu/wpi/first/util" / f,
            ftclib_root / "core/src/test/java/com/arcrobotics/ftclib/util" / f,
        )


def copy_upstream_src(ftclib_root):
    ftclib_root = Path(ftclib_root)
    wpilib_root = Path(".")

    print(wpilib_root)
    print(ftclib_root)

    copy_geometry(wpilib_root, ftclib_root)
    copy_trajectory(wpilib_root, ftclib_root)
    copy_spline(wpilib_root, ftclib_root)
    copy_controller(wpilib_root, ftclib_root)
    copy_kinematics(wpilib_root, ftclib_root)
    copy_interpolation(wpilib_root, ftclib_root)
    copy_wpimath_utils(wpilib_root, ftclib_root)

    copy_wpiutil(wpilib_root, ftclib_root)


def main():
    name = "wpimath"
    url = "https://github.com/wpilibsuite/allwpilib.git"
    tag = "v2024.3.2"

    patch_options = {
        "use_threeway": True,
    }

    wpimath = Lib(name, url, tag, copy_upstream_src, patch_options)
    wpimath.main()


if __name__ == "__main__":
    main()
