# FRC-2022
The Rapid React (FRC 2022) Code

WinT-3794's code for the FIRST Robotics Competition 2022 (Rapid React). Written in Java and is based off WPILib's Java control system.

[![Robot CI](https://github.com/WinT-3794/FRC-2022/actions/workflows/main.yml/badge.svg)](https://github.com/WinT-3794/FRC-2022/actions/workflows/main.yml)

## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download Gradle and needed FRC/Vendor libraries (make sure you're using Java 11 or greater)
1. Run `./gradlew downloadAll` to download FRC tools (ShuffleBoard, etc.)
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension from the release page on [this repository](https://github.com/wpilibsuite/allwpilib/releases/latest)
2. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Conventions

k*** (i.e. kDriveWheelTrackWidthInches): Final constants, especially those found in the Constants.java file.

m*** (i.e. m_drivetrain): Private instance variables
