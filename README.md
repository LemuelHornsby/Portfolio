# ⚓ Lemuel Hornsby‑Odoi — Maritime Autonomy & Marine Systems Portfolio

> **Marine Engineer** specializing in **autonomous marine systems**, **ship maneuvering**, and **control engineering**.  
> Focus areas: **physics-based vessel models**, **Nonlinear MPC (NMPC)**, **Deep Reinforcement Learning (DRL)**, and **digital twins**—grounded in real engine-room operations and maritime industry constraints.

---

## Navigation Chart (Repository Overview)

This repository is organized as a set of project “modules” (folders), each documenting an engineering deliverable relevant to modern maritime operations:

- **Yacht Autonomous Docking Project/** — Hybrid DRL + NMPC autonomy stack for safe docking in confined waters  
- **Digital Twin Process for Pump System/** — Digital-twin workflow applied to a pump system (documentation + artifacts)  
- **Constraint Programming/** — Shipbuilding production scheduling optimization (constraint programming case study)  
- **CNC Machining with error detection/** — Manufacturing quality / error detection work (process & reporting)

---

## Flagship Project — Autonomous Docking (Hybrid DRL + NMPC)

### **Hybrid DRL + NMPC for Safe Autonomous Yacht Docking**
**Folder:** `Yacht Autonomous Docking Project/`

A full-stack autonomy pipeline for **low-speed, high-risk maneuvers** in marina conditions—designed to be **constraint-aware**, **safe**, and **operationally interpretable**.

**Core architecture**
- **3‑DOF physics-based maneuvering model (MMG)** for realistic vessel response
- **NMPC** for structured trajectory tracking under constraints (actuator-aware guidance)
- **DRL (PPO)** policy for reactive obstacle avoidance and recovery behaviors
- **Hybrid supervisor** to select the safer control mode in real time

**Why it matters (maritime relevance)**
- Docking is a **high-consequence** operation (damage risk, personnel risk, port congestion impact)
- Hybrid control improves **robustness + explainability** compared with pure learning approaches
- The Unity co-simulation loop supports **evidence generation**, digital validation, and future **HIL-ready** workflows

**Quick link**
- Project overview: `Yacht Autonomous Docking Project/README.md`

---

## Digital Twin Engineering — Pump System

### **Digital Twin Process for Pump System**
**Folder:** `Digital Twin Process for Pump System/`

A documentation-driven project focused on building and communicating a **digital twin process** for a pump system—aligned with reliability, monitoring, and lifecycle engineering perspectives common in marine auxiliary systems.

**Maritime value**
- Supports **condition monitoring**, **fault detection**, and **maintenance planning**
- Maps naturally to shipboard systems: ballast, bilge, cooling water, fuel transfer, firefighting, etc.
- Demonstrates a disciplined approach to **model ↔ data ↔ validation** workflows

**Evidence**
- Includes a detailed PDF report in `Digital Twin Process for Pump System/Documents/`

---

## Production & Yard Optimization — Constraint Programming

### **Shipbuilding Production Scheduling Optimization**
**Folder:** `Constraint Programming/Case Study/`

Constraint programming models for scheduling a multi-stage production flow (e.g., steel cutting → welding → assembly → finishing). The included Python levels show progressively richer modeling, starting from feasibility.

**What’s inside**
- Python scripts (`Level1.py` …) using **constraint programming** patterns:
  - alternative machine selection per stage
  - no-overlap constraints per machine
  - staged production structure scaffolding

**Maritime industry relevance**
- Directly applicable to **shipyard throughput**, **block construction sequencing**, resource bottlenecks, and lead-time reduction
- Bridges operations research with engineering execution—useful for modern digital shipyard initiatives

---

## Manufacturing / Quality Systems

### **CNC Machining with Error Detection**
**Folder:** `CNC Machining with error detection/`

A process and reporting focused project on machining with error detection / lean manufacturing context.

**Why it’s included in a maritime portfolio**
- Maritime engineering depends on reliable fabrication and inspection workflows
- Demonstrates systems thinking: process monitoring, quality control, and actionable reporting

---

## Tooling & Languages Aboard (Tech Stack)

This repo contains work across multiple languages and engineering environments, including:
- **Python** (simulation, optimization/constraint models, control/ML pipelines)
- **C / C#** (systems-level + tooling environments)
- Plus supporting documents and artifacts (PDF/PPTX/media)

---

## How to Use This Repo

1. Start with the **flagship autonomy project**:  
   `Yacht Autonomous Docking Project/README.md`
2. Review supporting engineering depth:
   - Pump system digital twin documentation
   - Shipbuilding scheduling optimization models
3. Check project folders for **documents, code, and visual evidence** (images/videos where included)

---

## About / Contact

**Lemuel Hornsby‑Odoi**  
Marine Engineer — Autonomous Marine Systems • Maneuvering & Control • Digital Twins  
MSc Thesis Project work documented in this portfolio (April 2026)

> If you’d like, tell me what contact links you want shown here (LinkedIn, email, personal site), and I’ll format them in the same naval-themed style.
