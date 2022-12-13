# 2022 Robot Code (New)

[![CI](https://github.com/frc604/FRC-2022-NEW/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/frc604/FRC-2022-NEW/actions/workflows/main.yml)

This is a clean version of the 2022 robot code, meant for learning in preparation for the 2023 season.

## Setup

### To enable autoformatting when saving a file:

- [Install](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-spotless-gradle) the `richardwillis.vscode-spotless-gradle` VS Code extension.

- In `File > Preferences > Settings`, search for `Format on Save` and enable it.

- If asked to select a formatter, choose `Spotless Gradle`.

## Code Structure

- `src/main/java/frc/quixlib`: General utilities to be used year-after-year
- `src/main/java/frc/robot`: Year-specific code
- `src/test/java/frc/...`: Unit tests for both `quixlib` and `robot`
- `offboard`: Code that doesn't run onboard the robot

## Commands

- `./gradlew spotlessApply` to format all code.
