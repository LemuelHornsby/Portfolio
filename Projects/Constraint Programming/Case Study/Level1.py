# CS1 – Level 1: Basic feasibility (no optimization)
# - Data is hard-coded (manually transcribed) to avoid Excel read.
# - Constraints:
#     * One stage per block (via alternative: pick exactly one machine per stage)
#     * One block per machine at a time (no_overlap on each machine)
# - No precedence and no objective (feasibility only).
# - Visualization with docplex.cp.utils_visu

from docplex.cp.model import CpoModel
import docplex.cp.utils_visu as visu

# -----------------------------
# Manual data (from the provided Excel)
# -----------------------------
# Blocks & types (Production_Order sheet)
BLOCKS = [
    ("BB_01", "BB"), ("BB_02", "BB"), ("BB_03", "BB"), ("BB_04", "BB"),
    ("BB_05", "BB"), ("BB_06", "BB"), ("BB_07", "BB"), ("BB_08", "BB"),
    ("SB_01", "SB"), ("SB_02", "SB"), ("SB_03", "SB"), ("SB_04", "SB"),
    ("SB_05", "SB"), ("SB_06", "SB"), ("SB_07", "SB"), ("SB_08", "SB"),
    ("SB_09", "SB"), ("SB_10", "SB"),
    ("DB_01", "DB"), ("DB_02", "DB"), ("DB_03", "DB"), ("DB_04", "DB"),
    ("DB_05", "DB"), ("DB_06", "DB"),
]

# Stages (Model_Summary): 4 sequential stages exist in the case;
# for Level 1 we do not add precedence constraints.
NUM_STAGES = 4

# Machine set per stage (Stage_Machine_Mapping)
MACHINES_BY_STAGE = {
    1: ["S1_M1", "S1_M2"],                 # Steel Cutting
    2: ["S2_M1", "S2_M2", "S2_M3"],        # Welding
    3: ["S3_M1", "S3_M2", "S3_M3"],        # Assembly (cranes are Level 3; ignored here)
    4: ["S4_M1"],                          # Finishing
}

# Processing times by block type & machine (Processing_Times sheet)
# (times are in the same units across all stages/machines)
PT = {
    # Bottom Blocks (BB)
    "BB": {
        "S1_M1": 6, "S1_M2": 7,
        "S2_M1": 8, "S2_M2": 9, "S2_M3": 7,
        "S3_M1": 12, "S3_M2": 14, "S3_M3": 13,
        "S4_M1": 4,
    },
    # Side Blocks (SB)
    "SB": {
        "S1_M1": 4, "S1_M2": 5,
        "S2_M1": 6, "S2_M2": 7, "S2_M3": 5,
        "S3_M1": 8,  "S3_M2": 9,  "S3_M3": 8,
        "S4_M1": 3,
    },
    # Deck Blocks (DB)
    "DB": {
        "S1_M1": 5, "S1_M2": 6,
        "S2_M1": 7, "S2_M2": 8, "S2_M3": 6,
        "S3_M1": 10, "S3_M2": 11, "S3_M3": 11,
        "S4_M1": 3,
    },
}

# -----------------------------
# Build CPO model
# -----------------------------
mdl = CpoModel(name="CS1_Level1_Feasibility")

# Mandatory interval per (block, stage)
op = {}  # op[(block_id, stage)] -> interval_var (no fixed size to allow machine-dependent times)
for b_id, b_type in BLOCKS:
    for s in range(1, NUM_STAGES + 1):
        op[(b_id, s)] = mdl.interval_var(name=f"{b_id}_S{s}")

# Optional machine-specific intervals + link via 'alternative'
op_m = {}  # op_m[(block_id, stage, machine_id)]
for b_id, b_type in BLOCKS:
    for s in range(1, NUM_STAGES + 1):
        for m_id in MACHINES_BY_STAGE[s]:
            # Defensive check: ensure processing time exists for this block type + machine
            if b_type not in PT or m_id not in PT[b_type]:
                raise KeyError(
                    f"Processing time missing for block type '{b_type}' on machine '{m_id}' "
                    f"(block {b_id}, stage {s}). Please add entry to PT."
                )
            dur = PT[b_type][m_id]
            op_m[(b_id, s, m_id)] = mdl.interval_var(
                name=f"{b_id}_S{s}_{m_id}",
                optional=True,
                size=dur
            ) 
        # Exactly one machine is chosen for this (block, stage), sharing start/end with the mandatory op
        mdl.add(
            mdl.alternative(
                op[(b_id, s)],
                [op_m[(b_id, s, m_id)] for m_id in MACHINES_BY_STAGE[s]]
            )
        )

# No-overlap on each machine: never process two blocks at once on the same machine
for s, machines in MACHINES_BY_STAGE.items():
    for m_id in machines:
        mdl.add(
            mdl.no_overlap([op_m[(b_id, s, m_id)] for (b_id, _bt) in BLOCKS])
        )

# NOTE (Level 1): No precedence constraints (S1 -> S2 -> S3 -> S4)
# NOTE (Level 1): No objective (feasibility only)

# -----------------------------
# Solve
# -----------------------------
print("Solving Level 1 (feasibility only)...")
msol = mdl.solve(TimeLimit=60)  # feasibility search

if not msol:
    print("No feasible schedule found.")
else:
    print("Solve status:", msol.get_solve_status())
    # -----------------------------
    # Print a concise schedule summary: which machine was chosen per (block,stage)
    # and a per-machine ordered list of assigned intervals.
    # -----------------------------
    print("\nSchedule summary (per block stage -> chosen machine, start, duration):")
    machine_sched = {m: [] for machines in MACHINES_BY_STAGE.values() for m in machines}
    for (b_id, b_type) in BLOCKS:
        for s in range(1, NUM_STAGES + 1):
            chosen = None
            for m_id in MACHINES_BY_STAGE[s]:
                itv = msol.get_var_solution(op_m[(b_id, s, m_id)])
                if itv and itv.is_present():
                    start = itv.get_start()
                    dur = itv.get_length()
                    end = itv.get_end()
                    print(f"  {b_id} S{s}: {m_id}  start={start}  dur={dur}  end={end}")
                    machine_sched[m_id].append((start, b_id, s, dur))
                    chosen = m_id
                    break
            if chosen is None:
                print(f"  {b_id} S{s}: <no machine present in solution>")

    print("\nSchedule summary (per machine, ordered by start):")
    for m_id in sorted(machine_sched.keys()):
        tasks = sorted(machine_sched[m_id], key=lambda x: x[0])
        print(f"  {m_id} ({len(tasks)} tasks):")
        for start, b_id, s, dur in tasks:
            print(f"    start={start:>3}  dur={dur:>3}  {b_id}_S{s}")
    # -----------------------------
    # Visualization by machine
    # -----------------------------
    visu.timeline("CS1 – Level 1 Feasible Schedule (by Machine)")
    for s in range(1, NUM_STAGES + 1):
        for m_id in MACHINES_BY_STAGE[s]:
            visu.sequence(f"{m_id}")
            for (b_id, b_type) in BLOCKS:
                itv = msol.get_var_solution(op_m[(b_id, s, m_id)])
                if itv and itv.is_present():
                    visu.interval(itv, s, f"{b_id}_S{s}")
    visu.show()
