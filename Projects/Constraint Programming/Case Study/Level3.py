# CS1 – Level 3 (Enhanced): Shared cranes with tighter horizon, robust checks, and impact analysis
# Requires: docplex (>=2.25.222)
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

# -----------------------------
# Level 3: Crane resource settings
# -----------------------------
CRANE_STAGE = 3  # cranes are used in Stage 3
CRANE_CAPACITY = 2  # total number of cranes available

# If not all S3 machines or types need cranes, adjust below.
# Example: S3_M3 does not need cranes => set demand to 0.
CRANE_DEMAND_BY_MACHINE = {
    "S3_M1": 1,
    "S3_M2": 1,
    "S3_M3": 1,
}


def needs_crane_type(block_type: str) -> bool:
    """Return True if this block type needs cranes during Stage 3.
    Refine if only some types require cranes (e.g., return block_type != "SB")."""
    return True


def crane_demand_for(block_type: str, machine_id: str) -> int:
    """Return crane demand for a given (block type, machine) combo during Stage 3.
    Default: 1 for all S3 machines and all types; override tables above to refine."""
    if not needs_crane_type(block_type):
        return 0
    return int(CRANE_DEMAND_BY_MACHINE.get(machine_id, 1))


# -----------------------------
# Helpers: computed horizon upper bound
# -----------------------------
def compute_horizon_upper_bound() -> int:
    """Compute a safe schedule horizon upper bound.
    For each block, sum of per-stage max durations among machines; then sum across blocks.
    """
    max_pt_by_stage_type = {s: {} for s in range(1, NUM_STAGES + 1)}
    # Compute max PT per (stage, type)
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

    # Sum upper bound across blocks
    total = 0
    for _bid, btype in BLOCKS:
        total += sum(max_pt_by_stage_type[s][btype] for s in range(1, NUM_STAGES + 1))
    # Add a small slack to be safe
    return int(total + 1)


# -----------------------------
# Build model
# -----------------------------
mdl = CpoModel(name="CS1_Level3_Cranes_Enhanced")

# Mandatory intervals per (block, stage) – sized via chosen machine
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
            iv = mdl.interval_var(name=f"{b_id}_S{s}_{m_id}", optional=True, size=dur)
            op_m[(b_id, s, m_id)] = iv
            opts.append(iv)
        mdl.add(mdl.alternative(op[(b_id, s)], opts))

# No-overlap per machine
for s, machines in MACHINES_BY_STAGE.items():
    for m_id in machines:
        mdl.add(mdl.no_overlap([op_m[(b_id, s, m_id)] for b_id, _t in BLOCKS]))

# Precedence (S1 -> S2 -> S3 -> S4)
for b_id, _t in BLOCKS:
    for s in range(1, NUM_STAGES):
        mdl.add(mdl.end_before_start(op[(b_id, s)], op[(b_id, s + 1)]))

# Objective: minimize makespan
last_stage = NUM_STAGES
makespan = mdl.max([mdl.end_of(op[(b_id, last_stage)]) for b_id, _t in BLOCKS])
mdl.add(mdl.minimize(makespan))

# Level 3: Shared cranes as a cumulative resource using optional machine intervals
H = compute_horizon_upper_bound()
pulses = []
for b_id, b_type in BLOCKS:
    # Only Stage 3 uses cranes
    s = CRANE_STAGE
    for m_id in MACHINES_BY_STAGE[s]:
        demand = crane_demand_for(b_type, m_id)
        if demand > 0:
            pulses.append(mdl.pulse(op_m[(b_id, s, m_id)], demand))

if pulses:
    crane_usage = mdl.sum(pulses)
    mdl.add(mdl.always_in(crane_usage, 0, H, 0, CRANE_CAPACITY))


# -----------------------------
# Solve
# -----------------------------
print("Solving Level 3 Enhanced (precedence + makespan + cranes)...")
msol = mdl.solve(TimeLimit=120)

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
    # Impact analysis: crane concurrency
    # -----------------------------
    events = []  # (time, delta, label)
    s = CRANE_STAGE
    for b_id, b_type in BLOCKS:
        for m_id in MACHINES_BY_STAGE[s]:
            demand = crane_demand_for(b_type, m_id)
            if demand <= 0:
                continue
            itv = msol.get_var_solution(op_m[(b_id, s, m_id)])
            if itv and itv.is_present():
                st = itv.get_start()
                en = itv.get_end()
                events.append((st, +demand, (b_id, s, m_id)))
                events.append((en, -demand, (b_id, s, m_id)))

    events.sort(key=lambda x: (x[0], -x[1]))

    peak = 0
    cur = 0
    peak_times = []  # list of (start, end) where utilization equals peak
    at_capacity = []  # list of (start, end) where utilization == CRANE_CAPACITY
    last_time = None
    last_level = None
    for i, (t, delta, _lbl) in enumerate(events):
        if last_time is not None and t > last_time and last_level is not None:
            # Record the interval [last_time, t) at level last_level
            if last_level == peak and t > last_time:
                peak_times.append((last_time, t))
            if last_level == CRANE_CAPACITY and t > last_time:
                at_capacity.append((last_time, t))
        # Apply change
        cur += delta
        last_time = t
        last_level = cur
        if cur > peak:
            peak = cur

    # Print impact summary
    print("\nCrane usage impact analysis:")
    print(f"  Capacity: {CRANE_CAPACITY}")
    print(f"  Peak concurrent crane demand: {peak}")
    if at_capacity:
        total_cap_time = sum(en - st for st, en in at_capacity)
        print(f"  Time at full capacity: {total_cap_time}")
        # Show first few windows at capacity
        show = at_capacity[:5]
        for st, en in show:
            print(f"    At capacity during [{st}, {en})")
    else:
        print("  Never hits full capacity.")

    # Identify one peak moment and list overlapping blocks
    if peak > 0 and events:
        # Find first time where level equals peak
        cur = 0
        active = set()
        peak_moment = None
        for t, delta, lbl in events:
            # Before applying delta, interval [prev_t, t) was at 'cur'
            cur += delta
            if delta > 0:
                active.add(lbl)
            else:
                # remove by label
                try:
                    active.remove(lbl)
                except KeyError:
                    pass
            if cur == peak and peak_moment is None:
                peak_moment = t
                break
        if peak_moment is not None:
            # Collect tasks covering peak moment from the solution
            overlapping = []
            for b_id, b_type in BLOCKS:
                itv_best = None
                for m_id in MACHINES_BY_STAGE[CRANE_STAGE]:
                    demand = crane_demand_for(b_type, m_id)
                    if demand <= 0:
                        continue
                    itv = msol.get_var_solution(op_m[(b_id, CRANE_STAGE, m_id)])
                    if itv and itv.is_present():
                        if itv.get_start() <= peak_moment < itv.get_end():
                            itv_best = (m_id, itv)
                            break
                if itv_best:
                    m_id, itv = itv_best
                    overlapping.append((itv.get_start(), b_id, m_id, itv.get_length()))
            overlapping.sort(key=lambda x: x[0])
            print(f"  Example peak moment at t={peak_moment}, overlapping crane ops:")
            for st, b_id, m_id, dur in overlapping[:10]:
                print(f"    start={st:>3}  dur={dur:>3}  {b_id}_S{CRANE_STAGE}_{m_id}")

    # -----------------------------
    # Visualization (by machine)
    # -----------------------------
    visu.timeline("CS1 – Level 3 Enhanced (by Machine, cranes on Stage 3)")
    for s in range(1, NUM_STAGES + 1):
        for m_id in MACHINES_BY_STAGE[s]:
            visu.sequence(f"{m_id}")
            for b_id, b_type in BLOCKS:
                itv = msol.get_var_solution(op_m[(b_id, s, m_id)])
                if itv and itv.is_present():
                    label = f"{b_id}_S{s}"
                    if s == CRANE_STAGE and crane_demand_for(b_type, m_id) > 0:
                        label += " [crane]"
                    visu.interval(itv, s, label)
    visu.show()
