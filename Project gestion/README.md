# Project Management — Lead: Mickaël Borges

## About

To ensure the successful development of the robot, various project management documents were created to clearly define and scope the project. These files are categorized into subfolders (e.g., planning, meeting minutes, systems engineering analysis), while the project specifications (*Cahier des Charges*), bill of materials (BOM), and budget files are located directly at the root of this directory.

---

## Functional & Systems Engineering Analysis
**Contributors:** Maxime Fabre | Mickaël Borges | Thibaut Martin

To perform the functional and systems engineering analysis, we utilized **Capella**, an open-source MBSE (Model-Based Systems Engineering) application developed by the Eclipse Foundation.

Various diagrams were constructed to structure the design workflow, identify key system requirements, and map project interfaces:

* **Operational Architecture Blank (OAB)** — *Created by Thibaut Martin*
  Identifies external entities, environment actors, and operational capabilities interacting with the robot.
  
  *(Insert OAB Image Here)*

* **Missions Capabilities Blank (MCB)** — *Created by Mickaël Borges*
  Outlines the high-level missions required for the competition alongside the functional capabilities needed to achieve them.
  
  *(Insert MCB Image Here)*

* **Physical Architecture Blank (PAB)** — *Created by Maxime Fabre*
  Defines how physical components implement the required functional capabilities.
  
  *(Insert PAB Image Here)*

Additionally, a comprehensive summary table prioritizing task execution based on system requirements was compiled by Maxime Fabre.
* **File Location:** `[Insert Table File Path]`

---

## Budget & Component Selection
**Contributors:** Mickaël Borges | Thibaut Martin


The functional analysis provided the necessary technical requirements to select hardware components. Comparative studies were conducted to determine the optimal trade-off between performance, cost, and availability.

* Detailed component evaluations can be found in `[Insert Component List File Name]`.
* The overall budget accounts for:
  * Newly purchased hardware and components.
  * 3D printing filament consumption.
  * Reused inventory and assets owned by the association.
  * Consumables (solder, wiring, electrical power usage, etc.).

**Estimated Project Budget:** `[Insert Budget Amount]`

---

## Risk Analysis (FMEA / AMDEC) 
**Contributors:** Ryan Gedoir


A proactive risk analysis was conducted using the **FMEA (Failure Mode and Effects Analysis / AMDEC)** methodology to identify failure points, evaluate severity, and plan mitigation strategies.

* **Primary Critical Risk Identified:** Failing the official competition homologation process.
* **Mitigation Strategy:** Implementation of a strict internal pre-homologation checkpoint prior to the competition.

Detailed risk metrics and action plans are documented in `[Insert Risk Analysis File Name]`.

---

## Schedule & Project Planning 
**Contributors:** Mickaël Borges
Project timelines, task dependencies, and milestones were structured using a Gantt chart created with **GanttProject** (by Bar political / ADAE).

*(Insert Gantt Chart Image Here)*

* **File Location:** `[Insert Folder/File Path]`
* **Resource Allocation:** Each task within the Gantt chart is explicitly assigned to a designated team member to ensure accountability.

---

## Project Specifications (Cahier des Charges) 
**Contributors:** Maxime Fabre

The Project Specifications document establishes the official technical baseline for the robot. It translates the Eurobot / French Robotics Cup rulebook into concrete engineering constraints and performance objectives.

### Key Focus Areas:
1. **Regulatory Compliance:** Strict adherence to dimensions, safety margins, perimeter rules, and mandatory safety features (e.g., emergency stop button, starting cord).
2. **Game Objectives:** Detailed functional requirements for playing field tasks, game element manipulation, and scoring strategies within the 100-second match limit.
3. **Performance Metrics:** Speed, positioning accuracy, navigation reliability, and power autonomy specifications.
4. **Constraint Tracking:** Boundaries for total mass, maximum power consumption, and manufacturing budget constraints.

* **File Location:** `[Insert Specification Document Name/Path]`