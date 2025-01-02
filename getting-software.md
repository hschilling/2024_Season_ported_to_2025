# Getting and installing software for FRC Programmers

Here are links for getting the software needed for the season. 

## For a Drivers Station Computer (Windows Only)

### FRC Game Tools
FRC Game Tools is a software bundle that includes the FRC Driver Station and FRC Utilities. These components are required for FRC teams to configure and control robots and communicate with the field.

Available for download from https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html.

### WPILib Tools
Follow the directions here https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html but when the option is given between "Everything" and Tools Only", select "Tools Only". The drivers station doesn't need the full development environment, just the Tools like Shuffleboard and Elastic. 

## REV tools
Instructions on how to get the 2025 REV Tools and firmware are given at https://github.com/wpilibsuite/2025Beta/blob/main/REV.md.

## CTRE Tools
Phoenix Tuner X is available through app stores. See https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/index.html for more information.

## For a Development Computer (Windows and Mac)

### WPILib Tools
Follow the directions here https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html. When the option is given between "Everything" and Tools Only", select "Everything". 

This will also install:

- Shuffleboard
- AdvantageScope
- Elastic
- Choreo

## Third-party libraries
All vendordep operations can be controlled by the new Dependency Manager. Click the WPILib logo in the activity bar on the left to access it. See [Installing Libraries](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html#installing-libraries) for more information.

This will let you install these needed libraries:

- CTRE-Phoenix (v6) - for the Pigeon
- PathplannerLib
- REVLib
- ChoreoLib (if the team uses Choreo)
- photonlib (if the team uses PhotonVision)

## PathPlanner
The PathPlanner application is not included with the WPILib installation like the other tools. See https://pathplanner.dev/home.html for information on how to install it.

# Third-party library docs
- CTRE-Phoenix (v6)
    - https://v6.docs.ctr-electronics.com/en/stable/
    - https://api.ctr-electronics.com/phoenix6/latest/java/
- PathplannerLib
    - https://pathplanner.dev/ 
    - https://pathplanner.dev/api/java/
- REVLib
    - https://codedocs.revrobotics.com/java/
    - https://docs.revrobotics.com/brushless/revlib/revlib-overview
    - Code examples https://github.com/REVrobotics/REVLib-Examples

