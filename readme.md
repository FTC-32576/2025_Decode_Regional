
**🌎 Available languages:**
[![English](https://img.shields.io/badge/lang-en-red.svg)](readme.md)
[![Português](https://img.shields.io/badge/lang-pt--br-green.svg)](readme.pt.md)

# Java Is All Might

This repository contains the official software for **FTC Team #32576 — All Might**, developed for the current **FIRST Tech Challenge (FTC)** season.
Our project is based on the **NGC — Next Generation Codebase**, an internal architecture model designed by the team to keep our codebase clean, scalable, and easy to maintain across multiple seasons.

---

## 🧩 What Is NGC (Next Generation Codebase)?

**NGC** defines the organizational and architectural standards used by Team All Might.
It separates **generic, reusable code** from **season-specific logic**, allowing the same base to evolve with minimal refactoring from year to year.

This structure enables:

* 🔁 **Code reusability** across seasons (e.g., PID, IMU, Limelight utilities)
* 🧱 **Modularity** — every robot subsystem is self-contained
* ⚙️ **Ease of maintenance** and debugging
* 🚀 **Consistency** among developers and subteams

> 💡 **NGC Insight:**
> By isolating *Robot Code* from *Library Code*, strategy or hardware changes never affect the underlying control and sensor logic — making the entire system more sustainable and scalable.

---

## ⚙️ Code Organization

| Category           | Robot Code (Season Specific)                                    | Library Code (TeamLib / Java_Is_AllMight)             |
| ------------------ | --------------------------------------------------------------- | ----------------------------------------------------- |
| **Purpose**        | Implements logic for the current season                         | Provides reusable tools, algorithms, and abstractions |
| **Namespace**      | `ftc.team.allmight.plusultra.teamcode`                          | `ftc.team.Java_Is_AllMight`                           |
| **Responsibility** | Controls the robot, defines strategies, and autonomous routines | Supplies reusable and robust code for multiple years  |
| **Code Quality**   | Must be functional, tested, and competition-ready               | Must be clean, documented, and maintainable           |
| **Lifecycle**      | Lasts for a single season                                       | Persists across multiple seasons                      |

---

## 🗂️ Project Structure

```text
TeamCode/
│
├── Java/ftc/team/
│
├── Java_Is_AllMight/                # TeamLib — Reusable Library Code
│   ├── Control/                     # PID control, motion logic, navigation
│   ├── Sensors/                     # IMUHelper, LimelightHelper, and sensors
│   ├── Logging/                     # Logging and diagnostics
│
├── allmight/plusultra/teamcode/     # Robot Code — Current Season
│   ├── TeleOp/                      # Manual OpModes
│   ├── Auto/                        # Autonomous OpModes
│   └── Subsystems/                  # Robot subsystems (Drive, Shooter, Intake)
│
└── build.gradle                     # FTC SDK / Gradle configuration
```

---

## 🧠 TeamLib — Java_Is_AllMight

**TeamLib** is the backbone of the entire codebase.
It contains all reusable and hardware-independent modules that can be shared across seasons.

### Structure Overview

* **Control/**

  * PIDConfig, PIDController
* **Sensors/**

  * `IMUHelper`, `LimelightHelper`
* **Logging/**

  * Classes for data capture, diagnostics, and debugging

> 📘 **TeamLib Mission:**
> “Write once, use forever.”
> Everything that can be generalized should live in TeamLib, ready to be extended or inherited in future seasons.

---

## ⚙️ Robot Code

The **Robot Code** is where all season-specific logic lives — including OpModes, strategies, and subsystem management.

```
┌───────────────────┐
│ OpMode            │
│ (TeleOp / Auto)   │
└───────────────────┘
         │
┌────────▼────────┐ 
│ Robot Subsystems│ 
│ (Drive, Shooter…)│ 
└────────┬────────┘ 
         │
┌────────▼────────────┐ 
│ TeamLib             │ 
│ Control | Sensors    │
└────────┬────────────┘
         │
┌────────▼────────────┐
│ HardwareMap         │
│ Motors, Sensors, IMU│
└─────────────────────┘
```

* **OpMode:** Entry point for operation (manual or autonomous).
* **Subsystems:** Manage specific robot mechanisms.
* **TeamLib:** Provides advanced helpers, control algorithms, and utilities.
* **HardwareMap:** Direct interface between software and hardware.

> 🧩 Each subsystem leverages TeamLib utilities (PID, IMU, Limelight, etc.) to handle complex behavior while keeping logic focused and readable.

---

## 🔹 Continuous Integration (CI) & Code Quality

Team All Might maintains a **CI/CD (Continuous Integration / Continuous Deployment)** pipeline to ensure code reliability and maintain professional standards.

### 🔄 Automated Pipeline Includes:

* **✅ Validation:**
  Each *push* or *pull request* triggers a GitHub Actions workflow that checks:

  * Package and naming structure
  * FTC Gradle build
  * Code style and documentation compliance

* **🧪 Unit & Integration Tests:**
  TeamLib components such as PIDController, IMUHelper, and LimelightHelper undergo simulation-based testing to validate their mathematical behavior.

* **📦 Automated Deploy (optional):**
  When a stable build is tagged as a *release*, the pipeline automatically generates a signed `.apk` for the **Robot Controller (RC)**.

> 💡 This ensures that **no unverified change ever reaches the field**, improving reliability and reducing runtime issues during matches.

---

## 🧱 Development Philosophy

> “Our codebase is divided into two distinct layers: **robot code** and **library code**.
> The robot code defines behavior and strategy, while the library code provides reusable, tested systems for multiple seasons.”

This philosophy promotes:

* 🔧 **Simplified maintenance**
* ♻️ **Cross-season reusability**
* 📚 **Clear and unified documentation**
* 🚀 **Continuous technical evolution of Team All Might**

---

## 👑 Credits

Developed by **FTC Team #32576 — All Might**
**Software Leadership:** [**Nobre**](https://github.com/meuNobre)

Based on the **Next Generation Codebase (NGC)** architecture.
