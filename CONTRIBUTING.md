# Contributing to Rosdroid 🤖

Thank you for your interest in contributing to **Rosdroid**! We welcome contributions from developers, roboticists, students, and open-source enthusiasts of all skill levels.

---

## 📑 Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Suggesting Features & Hardware Support](#suggesting-features--hardware-support)
  - [Improving Documentation](#improving-documentation)
  - [Submitting Pull Requests](#submitting-pull-requests)
- [Development Setup](#development-setup)
- [Git Commit Guidelines](#git-commit-guidelines)
- [Coding Standards](#coding-standards)
- [Community & Communication](#community--communication)

---

## Code of Conduct

This project and everyone participating in it is governed by the [Rosdroid Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [opensource@botforge.org](mailto:opensource@botforge.org).

---

## How Can I Contribute?

### Reporting Bugs

Before creating bug reports, please check the [existing issues](https://github.com/botforge-robotics/ros2_android/issues) to ensure the issue hasn't already been reported.

When creating a bug report, please include:
- **Device Model & Android Version** (e.g. Google Pixel 7, Android 14)
- **App Version / Git Commit**
- **Detailed Steps to Reproduce**
- **Logcat Output**: Capture with `adb logcat -s Rosdroid:V RobotCoreService:V`
- **DDS / Network Setup**: Wi-Fi router model, subnet, whether Docker bridges are present on your PC.

### Suggesting Features & Hardware Support

We welcome proposals for new sensors, robotics algorithms, or microcontroller bridges.
1. Open an issue using the [Feature Request Template](.github/ISSUE_TEMPLATE/feature_request.md).
2. Clearly describe the robotics use-case and expected ROS 2 topic/message types.
3. Discuss with maintainers before submitting large structural changes.

### Improving Documentation

Documentation improvements, translation, tutorials, and robotics project showcases are hugely appreciated! Feel free to submit PRs directly for README and documentation fixes.

---

## Development Setup

### Prerequisites
- **Operating System**: Linux (Ubuntu 22.04 / 24.04 recommended) or macOS
- **Java Development Kit**: OpenJDK 17 (`sudo apt install openjdk-17-jdk`)
- **Android SDK & NDK**: Command-line tools or Android Studio Ladybug/Koala
- **CMake**: Version 3.22.1+
- **Android Device**: Physical `arm64-v8a` device with USB debugging enabled

### Building from Source

```bash
# Clone the repository
git clone https://github.com/botforge-robotics/ros2_android.git
cd ros2_android/rosdroid_app

# Build debug APK
./gradlew assembleDebug

# Install on connected device via ADB
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

### Running Unit Tests

```bash
cd rosdroid_app
./gradlew testDebugUnitTest
```

---

## Submitting Pull Requests

1. **Fork** the repository on GitHub.
2. **Clone** your fork locally:
   ```bash
   git clone https://github.com/<your-username>/ros2_android.git
   cd ros2_android
   ```
3. **Create a topic branch**:
   ```bash
   git checkout -b feature/my-robot-feature
   ```
4. **Make your changes** following our coding standards.
5. **Verify** builds and tests pass cleanly:
   ```bash
   cd rosdroid_app && ./gradlew assembleDebug testDebugUnitTest
   ```
6. **Commit** your changes with clear semantic messages.
7. **Push** to your fork:
   ```bash
   git push origin feature/my-robot-feature
   ```
8. **Open a Pull Request** against the primary repository branch with a clear description of the problem solved.

---

## Git Commit Guidelines

We follow [Conventional Commits](https://www.conventionalcommits.org/):

- `feat:` A new feature (e.g. `feat(sensor): add barometer sensor publisher`)
- `fix:` A bug fix (e.g. `fix(dds): prevent getifaddrs segfault in PRoot`)
- `docs:` Documentation only changes
- `style:` Formatting, missing semicolons, etc.
- `refactor:` Code change that neither fixes a bug nor adds a feature
- `perf:` Performance improvements
- `test:` Adding or updating tests
- `ci:` Changes to CI/CD workflows and scripts

---

## Coding Standards

### Kotlin & Android
- Use Jetpack Compose for all UI screens.
- Keep background robotics services strictly in foreground services (`RobotCoreService`).
- Avoid unnecessary thread blocking on the main UI dispatcher; use Kotlin Coroutines (`Dispatchers.IO` / `Dispatchers.Default`).

### C++ Native Code
- Native code is located in `rosdroid_app/app/src/main/cpp/`.
- Ensure memory safety, zero memory leaks, and bounds checking.
- Keep JNI boundary interactions lean; batch telemetry data where appropriate.

### Shell Scripts
- Write POSIX-compliant bash scripts with strict error handling (`set -e` or conditional checks).

---

## Community & Communication

- **GitHub Discussions**: For general questions, robotics showcases, and Q&A.
- **GitHub Issues**: For bug tracking and feature proposals.
- **Email**: [opensource@botforge.org](mailto:opensource@botforge.org)
