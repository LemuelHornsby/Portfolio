# CS1 – Level 4 (Enhanced): sequence-dependent setups + cranes, with comparison runtime
# Extended from Level 3: cranes + precedence + makespan
# Uses manual data transcription for simplicity
# Compares two models: with and without sequence-dependent setups

from docplex.cp.model import CpoModel
import docplex.cp.utils_visu as visu

# -----------------------------
# Manual data (transcribed)
# -----------------------------
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

NUM_STAGES = 4

# Machines per stage
MACHINES_BY_STAGE = {
    1: ["S1_M1", "S1_M2"],  # Steel Cutting
    2: ["S2_M1", "S2_M2", "S2_M3"],  # Welding
    3: ["S3_M1", "S3_M2", "S3_M3"],  # Assembly (cranes used here)
    4: ["S4_M1"],  # Finishing
}

# Processing time by type & machine
PT = {
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

# Sequence-dependent setup matrices by stage (type-to-type)
TYPES = ["BB", "SB", "DB"]
TYPE_INDEX = {t: i for i, t in enumerate(TYPES)}
SETUP_BY_STAGE = {
    1: [
        [0, 1, 2],
        [1, 0, 1],
        [2, 1, 0],
    ],
    2: [
        [0, 2, 3],
        [2, 0, 2],
        [3, 2, 0],
    ],
    3: [
        [0, 3, 4],
        [3, 0, 3],
        [4, 3, 0],
    ],
    4: [
        [0, 1, 1],
        [1, 0, 1],
        [1, 1, 0],
    ],
}

# Crane resource settings (Level 3+)
CRANE_STAGE = 3
CRANE_CAPACITY = 2
CRANE_DEMAND_BY_MACHINE = {
    "S3_M1": 1,
    "S3_M2": 1,
    "S3_M3": 1,
}


def needs_crane_type(block_type: str) -> bool:
    return True


def crane_demand_for(block_type: str, machine_id: str) -> int:
    if not needs_crane_type(block_type):
        return 0
    return int(CRANE_DEMAND_BY_MACHINE.get(machine_id, 1))


# Helpers: computed horizon upper bound


def compute_horizon_upper_bound() -> int:
    max_pt_by_stage_type = {s: {} for s in range(1, NUM_STAGES + 1)}
    for s in range(1, NUM_STAGES + 1):
        machines = MACHINES_BY_STAGE[s]
        for btype in PT.keys():
            durations = []
            for m in machines:
                dur = PT.get(btype, {}).get(m)
                if dur is None:
                    raise KeyError(
                        f"Missing PT for type '{btype}' on machine '{m}' (stage {s})."
                    )
                durations.append(dur)
            max_pt_by_stage_type[s][btype] = max(durations)
    total = 0
    for _bid, btype in BLOCKS:
        total += sum(max_pt_by_stage_type[s][btype] for s in range(1, NUM_STAGES + 1))
    return int(total + 1)


# Model builder


def build_model(enable_setups: bool):
    mdl = CpoModel(
        name=f"CS1_Level4_Enhanced_{'With' if enable_setups else 'No'}Setups"
    )

    # Master intervals per (block, stage)
    op = {
        (b_id, s): mdl.interval_var(name=f"{b_id}_S{s}")
        for b_id, _t in BLOCKS
        for s in range(1, NUM_STAGES + 1)
    }

    # Optional machine intervals + alternative link
    op_m = {}
    for b_id, b_type in BLOCKS:
        for s in range(1, NUM_STAGES + 1):
            opts = []
            for m_id in MACHINES_BY_STAGE[s]:
                dur = PT.get(b_type, {}).get(m_id)
                if dur is None:
                    raise KeyError(
                        f"Processing time missing for block type '{b_type}' on machine '{m_id}' (block {b_id}, stage {s})."
                    )
                iv = mdl.interval_var(
                    name=f"{b_id}_S{s}_{m_id}", optional=True, size=dur
                )
                op_m[(b_id, s, m_id)] = iv
                opts.append(iv)
            mdl.add(mdl.alternative(op[(b_id, s)], opts))

    # Precedence
    for b_id, _t in BLOCKS:
        for s in range(1, NUM_STAGES):
            mdl.add(mdl.end_before_start(op[(b_id, s)], op[(b_id, s + 1)]))

    # Capacity per machine: either setup-aware or simple no-overlap
    if enable_setups:
        TYPES_IDX = [TYPE_INDEX[_t] for (_b, _t) in BLOCKS]
        seq = {}
        for s, machines in MACHINES_BY_STAGE.items():
            setup_matrix = SETUP_BY_STAGE[s]
            for m_id in machines:
                machine_ops = [op_m[(b_id, s, m_id)] for (b_id, _t) in BLOCKS]
                seq[(s, m_id)] = mdl.sequence_var(
                    machine_ops, types=TYPES_IDX, name=f"SEQ_{m_id}"
                )
                mdl.add(mdl.no_overlap(seq[(s, m_id)], setup_matrix))
    else:
        for s, machines in MACHINES_BY_STAGE.items():
            for m_id in machines:
                mdl.add(
                    mdl.no_overlap([op_m[(b_id, s, m_id)] for (b_id, _t) in BLOCKS])
                )

    # Cranes (Stage 3), with computed horizon and machine/type-specific demand
    H = compute_horizon_upper_bound()
    pulses = []
    s = CRANE_STAGE
    for b_id, b_type in BLOCKS:
        for m_id in MACHINES_BY_STAGE[s]:
            demand = crane_demand_for(b_type, m_id)
            if demand > 0:
                pulses.append(mdl.pulse(op_m[(b_id, s, m_id)], demand))
    if pulses:
        crane_usage = mdl.sum(pulses)
        mdl.add(mdl.always_in(crane_usage, 0, H, 0, CRANE_CAPACITY))

    # Objective: minimize makespan
    last_stage = NUM_STAGES
    makespan = mdl.max([mdl.end_of(op[(b_id, last_stage)]) for (b_id, _t) in BLOCKS])
    mdl.add(mdl.minimize(makespan))

    return mdl, op, op_m


# Reporting helpers


def realized_setup_time(msol, op_m):
    """Compute realized total setup time across all machines using SETUP_BY_STAGE and types.
    Returns (total_setup, per_machine_dict). Only valid when setups are enabled.
    """
    total = 0
    per_machine = {}
    # For each machine, build realized order from present intervals (start times)
    for s, machines in MACHINES_BY_STAGE.items():
        setup_matrix = SETUP_BY_STAGE[s]
        for m_id in machines:
            # Collect present intervals with their block type and start
            present = []
            for b_id, b_type in BLOCKS:
                itv = msol.get_var_solution(op_m[(b_id, s, m_id)])
                if itv and itv.is_present():
                    present.append((itv.get_start(), b_type))
            present.sort(key=lambda x: x[0])
            # Sum setup times between consecutive tasks
            stime = 0
            for i in range(1, len(present)):
                from_t = TYPE_INDEX[present[i - 1][1]]
                to_t = TYPE_INDEX[present[i][1]]
                stime += setup_matrix[from_t][to_t]
            if stime > 0:
                per_machine[(s, m_id)] = stime
                total += stime
    return total, per_machine


# -----------------------------
# Run comparison
# -----------------------------
if __name__ == "__main__":
    # With setups
    print("Solving Level 4 Enhanced: WITH setups...")
    mdl_w, op_w, op_m_w = build_model(enable_setups=True)
    msol_w = mdl_w.solve(TimeLimit=150)
    if not msol_w:
        print("No solution found (WITH setups)")
    else:
        ms_w = None
        try:
            ms_w = msol_w.get_objective_value()
        except Exception:
            try:
                ms_w = msol_w.get_value(mdl_w.get_all_expressions()[0])
            except Exception:
                pass
        print("Makespan (WITH setups):", ms_w)
        total_setup_w, per_machine_w = realized_setup_time(msol_w, op_m_w)
        print("Total realized setup time (WITH):", total_setup_w)
        for (s, m_id), st in sorted(per_machine_w.items()):
            print(f"  Stage {s}, {m_id}: setup={st}")

    # Without setups
    print("\nSolving Level 4 Enhanced: WITHOUT setups...")
    mdl_n, op_n, op_m_n = build_model(enable_setups=False)
    msol_n = mdl_n.solve(TimeLimit=150)
    if not msol_n:
        print("No solution found (WITHOUT setups)")
    else:
        ms_n = None
        try:
            ms_n = msol_n.get_objective_value()
        except Exception:
            try:
                ms_n = msol_n.get_value(mdl_n.get_all_expressions()[0])
            except Exception:
                pass
        print("Makespan (WITHOUT setups):", ms_n)

    # Comparison summary
    if msol_w and msol_n:
        try:
            print("\nComparison summary:")
            print("  WITH setups   :", ms_w)
            print("  WITHOUT setups:", ms_n)
            if ms_w is not None and ms_n is not None:
                print("  Delta (WITH - WITHOUT):", ms_w - ms_n)
        except Exception:
            pass

    # Optional: simple visualization for WITH setups
    if msol_w:
        visu.timeline("CS1 – Level 4 Enhanced (WITH setups)")
        for s in range(1, NUM_STAGES + 1):
            for m_id in MACHINES_BY_STAGE[s]:
                visu.sequence(f"{m_id}")
                for b_id, b_type in BLOCKS:
                    itv = msol_w.get_var_solution(op_m_w[(b_id, s, m_id)])
                    if itv and itv.is_present():
                        label = f"{b_id}_S{s}"
                        if s == CRANE_STAGE and crane_demand_for(b_type, m_id) > 0:
                            label += " [crane]"
                        visu.interval(itv, s, label)
        visu.show()
