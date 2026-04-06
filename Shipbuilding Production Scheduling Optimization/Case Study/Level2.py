# CS1 – Level 2: Precedence constraints + makespan minimization
# - Data is hard-coded (manual transcription) to avoid Excel reads.
# - Adds:
#     * end_before_start between consecutive stages per block
#     * minimize makespan = max end time of last-stage ops
# - Keeps:
#     * alternative(...) for machine choice
#     * no_overlap(...) per machine
#     * timeline visualization (visu) by machine

from docplex.cp.model import CpoModel
import docplex.cp.utils_visu as visu

# -----------------------------
# Manual data (from Excel, transcribed)
# -----------------------------
# Blocks & types
BLOCKS = [
    ("BB_01", "BB"),
    ("BB_02", "BB"),
    ("BB_03", "BB"),
    ("BB_04", "BB"),
    ("BB_05", "BB"),
    ("BB_06", "BB"),
    ("BB_07", "BB"),
    ("BB_08", "BB"),
    ("SB_01", "SB"),
    ("SB_02", "SB"),
    ("SB_03", "SB"),
    ("SB_04", "SB"),
    ("SB_05", "SB"),
    ("SB_06", "SB"),
    ("SB_07", "SB"),
    ("SB_08", "SB"),
    ("SB_09", "SB"),
    ("SB_10", "SB"),
    ("DB_01", "DB"),
    ("DB_02", "DB"),
    ("DB_03", "DB"),
    ("DB_04", "DB"),
    ("DB_05", "DB"),
    ("DB_06", "DB"),
]

# Stages: 4 sequential stages (S1..S4)
NUM_STAGES = 4

# Machine set per stage
MACHINES_BY_STAGE = {
    1: ["S1_M1", "S1_M2"],  # Steel Cutting
    2: ["S2_M1", "S2_M2", "S2_M3"],  # Welding
    3: ["S3_M1", "S3_M2", "S3_M3"],  # Assembly
    4: ["S4_M1"],  # Finishing
}

# Processing times by block type & machine (same time unit for all)
PT = {
    # Bottom Blocks (BB)
    "BB": {
        "S1_M1": 6,
        "S1_M2": 7,
        "S2_M1": 8,
        "S2_M2": 9,
        "S2_M3": 7,
        "S3_M1": 12,
        "S3_M2": 14,
        "S3_M3": 13,
        "S4_M1": 4,
    },
    # Side Blocks (SB)
    "SB": {
        "S1_M1": 4,
        "S1_M2": 5,
        "S2_M1": 6,
        "S2_M2": 7,
        "S2_M3": 5,
        "S3_M1": 8,
        "S3_M2": 9,
        "S3_M3": 8,
        "S4_M1": 3,
    },
    # Deck Blocks (DB)
    "DB": {
        "S1_M1": 5,
        "S1_M2": 6,
        "S2_M1": 7,
        "S2_M2": 8,
        "S2_M3": 6,
        "S3_M1": 10,
        "S3_M2": 11,
        "S3_M3": 11,
        "S4_M1": 3,
    },
}

# -----------------------------
# Build CPO model
# -----------------------------
mdl = CpoModel(name="CS1_Level2_Precedence_Makespan")

# Mandatory interval per (block, stage)
# (no fixed size here; size comes from chosen machine via alternative)
op = {}
for b_id, b_type in BLOCKS:
    for s in range(1, NUM_STAGES + 1):
        op[(b_id, s)] = mdl.interval_var(name=f"{b_id}_S{s}")

# Optional machine-specific intervals + link via alternative(...)
op_m = {}
for b_id, b_type in BLOCKS:
    for s in range(1, NUM_STAGES + 1):
        options = []
        for m_id in MACHINES_BY_STAGE[s]:
            dur = PT[b_type][m_id]
            iv = mdl.interval_var(name=f"{b_id}_S{s}_{m_id}", optional=True, size=dur)
            op_m[(b_id, s, m_id)] = iv
            options.append(iv)
        mdl.add(mdl.alternative(op[(b_id, s)], options))

# No-overlap on each machine
for s, machines in MACHINES_BY_STAGE.items():
    for m_id in machines:
        mdl.add(mdl.no_overlap([op_m[(b_id, s, m_id)] for (b_id, _bt) in BLOCKS]))

# -----------------------------
# NEW in Level 2: Stage precedence within each block
# S1 -> S2 -> S3 -> S4
# -----------------------------
for b_id, _btype in BLOCKS:
    for s in range(1, NUM_STAGES):
        mdl.add(mdl.end_before_start(op[(b_id, s)], op[(b_id, s + 1)]))

# -----------------------------
# NEW in Level 2: Objective = minimize makespan
# -----------------------------
last_stage = NUM_STAGES
makespan = mdl.max([mdl.end_of(op[(b_id, last_stage)]) for (b_id, _bt) in BLOCKS])
mdl.add(mdl.minimize(makespan))

# -----------------------------
# Solve
# -----------------------------
print("Solving Level 2 (precedence + makespan)...")
msol = mdl.solve(TimeLimit=90)

if not msol:
    print("No solution found.")
else:
    print("Solve status:", msol.get_solve_status())
    try:
        print("Makespan:", msol.get_objective_value())
    except Exception:
        try:
            print("Makespan (eval):", msol.get_value(makespan))
        except Exception:
            pass

    # -----------------------------
    # Visualization by machine
    # -----------------------------
    visu.timeline("CS1 – Level 2 (by Machine)")
    for s in range(1, NUM_STAGES + 1):
        for m_id in MACHINES_BY_STAGE[s]:
            visu.sequence(f"{m_id}")
            for b_id, b_type in BLOCKS:
                itv = msol.get_var_solution(op_m[(b_id, s, m_id)])
                if itv and itv.is_present():
                    visu.interval(itv, s, f"{b_id}_S{s}")
    visu.show()
