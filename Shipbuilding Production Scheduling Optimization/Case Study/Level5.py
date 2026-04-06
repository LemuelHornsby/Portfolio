# CS1 – Level 5 (Enhanced): Due dates + setups + cranes with multi-objective comparison
# Extended from Level 4: due dates + weighted tardiness + lexicographic objective
# Uses manual data transcription for simplicity
# Compares two models: lexicographic (TWT then Cmax) vs Cmax-only

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
    1: ["S1_M1", "S1_M2"],
    2: ["S2_M1", "S2_M2", "S2_M3"],
    3: ["S3_M1", "S3_M2", "S3_M3"],
    4: ["S4_M1"],
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

# Sequence-dependent setups by stage (type-to-type)
TYPES = ["BB", "SB", "DB"]
TYPE_INDEX = {t: i for i, t in enumerate(TYPES)}
SETUP_BY_STAGE = {
    1: [[0, 1, 2], [1, 0, 1], [2, 1, 0]],
    2: [[0, 2, 3], [2, 0, 2], [3, 2, 0]],
    3: [[0, 3, 4], [3, 0, 3], [4, 3, 0]],
    4: [[0, 1, 1], [1, 0, 1], [1, 1, 0]],
}

# Crane resource settings
CRANE_STAGE = 3
CRANE_CAPACITY = 2
CRANE_DEMAND_BY_MACHINE = {"S3_M1": 1, "S3_M2": 1, "S3_M3": 1}


def needs_crane_type(block_type: str) -> bool:
    return True


def crane_demand_for(block_type: str, machine_id: str) -> int:
    if not needs_crane_type(block_type):
        return 0
    return int(CRANE_DEMAND_BY_MACHINE.get(machine_id, 1))


# Due dates and weights (set weights=1 for total tardiness)
DEFAULT_DUE_BY_TYPE = {"SB": 100, "DB": 110, "BB": 120}
WEIGHT_BY_TYPE = {"SB": 1, "DB": 3, "BB": 2}
DUE = {b_id: DEFAULT_DUE_BY_TYPE[b_type] for b_id, b_type in BLOCKS}

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


def build_model(objective_mode: str):
    """objective_mode: 'lex_twt_then_cmax' or 'cmax_only'"""
    mdl = CpoModel(name=f"CS1_Level5_Enhanced_{objective_mode}")

    # Master intervals per (block, stage)
    op = {
        (b_id, s): mdl.interval_var(name=f"{b_id}_S{s}")
        for b_id, _t in BLOCKS
        for s in range(1, NUM_STAGES + 1)
    }

    # Optional machine intervals + alternative link (with defensive PT checks)
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

    # Capacity with sequence-dependent setups
    TYPES_IDX = [TYPE_INDEX[_t] for (_b, _t) in BLOCKS]
    for s, machines in MACHINES_BY_STAGE.items():
        setup_matrix = SETUP_BY_STAGE[s]
        for m_id in machines:
            machine_ops = [op_m[(b_id, s, m_id)] for (b_id, _t) in BLOCKS]
            seq = mdl.sequence_var(machine_ops, types=TYPES_IDX, name=f"SEQ_{m_id}")
            mdl.add(mdl.no_overlap(seq, setup_matrix))

    # Cranes (Stage 3) with computed horizon and machine/type demand
    H = compute_horizon_upper_bound()
    pulses = []
    s3 = CRANE_STAGE
    for b_id, b_type in BLOCKS:
        for m_id in MACHINES_BY_STAGE[s3]:
            d = crane_demand_for(b_type, m_id)
            if d > 0:
                pulses.append(mdl.pulse(op_m[(b_id, s3, m_id)], d))
    if pulses:
        mdl.add(mdl.always_in(mdl.sum(pulses), 0, H, 0, CRANE_CAPACITY))

    # Objectives
    last_stage = NUM_STAGES
    makespan = mdl.max([mdl.end_of(op[(b_id, last_stage)]) for b_id, _t in BLOCKS])

    tard_terms = []
    for b_id, b_type in BLOCKS:
        completion = mdl.end_of(op[(b_id, last_stage)])
        tard = mdl.max([0, completion - DUE[b_id]])
        w = WEIGHT_BY_TYPE[b_type]
        tard_terms.append(w * tard)
    twt = mdl.sum(tard_terms)

    if objective_mode == "lex_twt_then_cmax":
        mdl.add(mdl.minimize_static_lex([twt, makespan]))
    elif objective_mode == "cmax_only":
        mdl.add(mdl.minimize(makespan))
    else:
        raise ValueError("Unknown objective_mode")

    return mdl, op, op_m, makespan, twt


# Reporting


def compute_per_block_tardiness(msol, op, last_stage):
    """Compute per-block tardiness using end times from the interval var solutions.
    Avoids calling .end_of() on interval variables (which is not a var method).
    """
    tard = []
    for b_id, _t in BLOCKS:
        itv = msol.get_var_solution(op[(b_id, last_stage)])
        if itv and itv.is_present():
            comp = itv.get_end()
        else:
            # Should not happen with mandatory intervals, but guard defensively
            comp = 0
        dd = DUE[b_id]
        tard.append((b_id, max(0, comp - dd), comp, dd))
    tard.sort(key=lambda x: x[1], reverse=True)
    return tard


# -----------------------------
# Run two solves and compare
# -----------------------------
if __name__ == "__main__":
    # (A) TWT then Cmax (lexicographic)
    print("Solving Level 5 Enhanced: TWT -> Cmax (lex)...")
    mdl_lex, op_lex, op_m_lex, makespan_lex, twt_lex = build_model("lex_twt_then_cmax")
    sol_lex = mdl_lex.solve(TimeLimit=180)

    ms_lex = None
    twt_val = None
    if sol_lex:
        try:
            ms_lex = sol_lex.get_value(makespan_lex)
            twt_val = sol_lex.get_value(twt_lex)
        except Exception:
            pass
        print("  Makespan (lex):", ms_lex)
        print("  Total (Weighted) Tardiness (lex):", twt_val)

        # Top tardy blocks
        per_block = compute_per_block_tardiness(sol_lex, op_lex, NUM_STAGES)
        print("  Top tardy blocks (up to 10):")
        for b_id, tard, comp, dd in per_block[:10]:
            print(f"    {b_id}: tard={tard} comp={comp} due={dd}")
    else:
        print("  No solution (lex)")

    # (B) Cmax only
    print("\nSolving Level 5 Enhanced: Cmax only...")
    mdl_cmax, op_cmax, op_m_cmax, makespan_cmax, twt_cmax = build_model("cmax_only")
    sol_cmax = mdl_cmax.solve(TimeLimit=180)

    ms_cmax = None
    twt_cmax_val = None
    if sol_cmax:
        try:
            ms_cmax = sol_cmax.get_value(makespan_cmax)
            twt_cmax_val = sol_cmax.get_value(twt_cmax)
        except Exception:
            pass
        print("  Makespan (cmax-only):", ms_cmax)
        print("  Total (Weighted) Tardiness (cmax-only):", twt_cmax_val)
    else:
        print("  No solution (cmax-only)")

    # Comparison summary
    if sol_lex and sol_cmax:
        print("\nComparison summary:")
        print("  Lex (TWT->Cmax):  twt=", twt_val, " cmax=", ms_lex)
        print("  Cmax-only     :  twt=", twt_cmax_val, " cmax=", ms_cmax)
        if (
            twt_val is not None
            and twt_cmax_val is not None
            and ms_lex is not None
            and ms_cmax is not None
        ):
            print(
                "  Delta (lex - cmax):  Δtwt=",
                twt_val - twt_cmax_val,
                " Δcmax=",
                ms_lex - ms_cmax,
            )

    # Optional visualization for lex solution
    if sol_lex:
        visu.timeline("CS1 – Level 5 Enhanced (TWT -> Cmax)")
        for s in range(1, NUM_STAGES + 1):
            for m_id in MACHINES_BY_STAGE[s]:
                visu.sequence(f"{m_id}")
                for b_id, b_type in BLOCKS:
                    itv = sol_lex.get_var_solution(op_m_lex[(b_id, s, m_id)])
                    if itv and itv.is_present():
                        label = f"{b_id}_S{s}"
                        if s == CRANE_STAGE and crane_demand_for(b_type, m_id) > 0:
                            label += " [crane]"
                        visu.interval(itv, s, label)
        visu.show()
