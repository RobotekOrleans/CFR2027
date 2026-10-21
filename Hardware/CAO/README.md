# CAD & Hardware - Robotek Orléans

## About

This section contains all 3D CAD modeling files for the robot's mechanical components. 3D modeling allows us to visualize the design clearly, check for potential mechanical constraints, and identify any assembly issues prior to manufacturing.

### Directory Structure

* **`full-assembly/`** *(or `nom/`)* — Contains the complete 3D assembly model of the robot.
* **Subfolders** — Dedicated directories for each individual mechanical component and subsystem.

### 3D Printing Setup

The association uses a **Bambu Lab X1 Carbon** 3D printer. The main materials used are **PLA** and **PETG**. The specific filament type used for each component is specified in the descriptions below.

---

## Components

### Pillars / Standoffs (`poteaux/`)

The robot structure relies on 4 main pillars, held together by threaded rods running through their center. Custom PLA pillars were 3D printed to distribute the structural load across a larger surface area compared to the threaded rods alone.

* **Location:** `poteaux/`
* **Material:** PLA
* **Filament Used:** ~239.6 g

---

### Motor Mounts (`support-moteur/`)

The robot is driven by 4 motors. Custom PLA mounting brackets were designed and 3D printed to secure the motors to the frame.

* **Location:** `support-moteur/`
* **Material:** PLA
* **Filament Used:** ~72.6 g

---

### Pump Mounts (`support-pompe/`)

The robot uses 4 vacuum/air pumps. Dedicated mounting brackets were 3D printed to securely fasten the pumps to the structure.

* **Location:** `support-pompe/`
* **Material:** PLA
* **Filament Used:** N/A *(To be determined)*