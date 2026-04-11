# mmg_setup_yacht.py
import math
from dataclasses import dataclass
from typing import Dict, Tuple

RHO_WATER = 1025.0  # seawater kg/m^3

# Utility functions. clamp is used to limit values within a range.This is particularly useful for ensuring that inputs to functions remain within acceptable bounds.
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

# Wrap angle to [-pi, pi]
def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

@dataclass
class YachtPrincipal:
    # Principal dimensions & properties
    L: float = 40.0     # [m] (use as Lpp for MMG baseline)
    B: float = 8.11     # [m]
    T: float = 2.22     # [m]
    disp_t: float = 363.77  # [tonnes]
    V_ref: float = 8.14  # [m/s] ~ 15.8 kn

    # LCG: 18m from FP; midship at 20m => xG = -2m (aft of midship)
    xG: float = -2.0    # [m] CG wrt midship (+fwd)

@dataclass
class Propeller:
    D: float = 1.5 # diameter [m]
    n_max_rps: float = 453.16 / 60.0  # [rev/s]
    w: float = 0.28 # wake fraction
    t_deduction: float = 0.18 # thrust deduction factor

    # KT(J) simple polynomial. Kt is thrust coefficient as a function of J (advance ratio).
    KT_a0: float = 0.52 # this represents the thrust coefficient at zero advance ratio.
    KT_a1: float = -0.52 # this coefficient represents the linear change in thrust coefficient with respect to the advance ratio.
    KT_a2: float = 0.0 # this coefficient represents the quadratic change in thrust coefficient with respect to the advance ratio.

@dataclass
class Rudder:
    A: float = 1.5  # [m^2]
    xR: float = -18.6   # [m] from midshipSSS
    max_deg: float = 35.0   # degrees

    CL_alpha: float = 4.0 # per rad.This value is known as "lift curve slope". It represents how much lift the rudder generates per radian of angle of attack.
    CD0: float = 0.03 # zero-lift drag coefficient. This represents the drag produced by the rudder when there is no lift (i.e., at zero angle of attack).
    k_induced: float = 0.08 # induced drag factor. This coefficient accounts for the additional drag generated due to the creation of lift by the rudder.

    k_eff: float = 0.75        # ↑ (mild) use effectiveness not huge area
    k_propwash: float = 0.5   # ↑ improves low-speed authority
    deadband_deg: float = 0.05  # tiny deadband to eliminate float noise


@dataclass
class BowThruster:
    # Simple bow thruster model (dormant unless commanded).
    # Generates a lateral force in body +Z (sway) direction at the bow.
    # Yaw moment is produced by the longitudinal lever arm about CG.
    # Convention: u:+X, v:+Z, r:+Y (Unity +X forward, +Z lateral, yaw about +Y)
    Y_max: float = 50000.0  # [N] peak lateral force at |bow_cmd|=1
    xBT: float = 16.0       # [m] thruster location along body +X from midship
    deadband: float = 0.01  # [-] command deadband

@dataclass
class HullDerivatives: # linear & nonlinear damping derivatives.Needed for calculating the forces acting on the hull of the yacht based on its motion in the water.
    Yv: float #this value represents the linear sway force derivative with respect to sway velocity. This value affects how the hull responds to lateral movements.
    Yr: float #this value represents the linear sway force derivative with respect to yaw rate. #This value influences the hull's response to rotational movements.
    Nv: float #this value represents the linear yaw moment derivative with respect to sway velocity. This value affects the rotational response of the hull to lateral movements.
    Nr: float #this value represents the linear yaw moment derivative with respect to yaw rate. This value influences the hull's rotational response to changes in yaw rate.
    Yvv: float #this value represents the nonlinear sway force derivative with respect to sway velocity squared. This value captures the nonlinear effects in the hull's lateral force response.
    Nrr: float #this value represents the nonlinear yaw moment derivative with respect to yaw rate squared. This value captures the nonlinear effects in the hull's rotational response.

@dataclass
class AddedMass: # added mass coefficients (dimensional)
    Xu_dot: float # Surge added mass coefficient. This value represents the additional mass that the hull appears to have in the surge direction due to the acceleration of the surrounding water.
    Yv_dot: float # Sway added mass coefficient. This value represents the additional mass that the hull appears to have in the sway direction due to the acceleration of the surrounding water.
    Nr_dot: float # Yaw added mass coefficient. This value represents the additional rotational inertia that the hull appears to have in yaw due to the acceleration of the surrounding water.

def estimate_Iz(pr: YachtPrincipal) -> float:
    m = pr.disp_t * 1000.0 # [kg]
    return 0.24 * m * pr.L**2 #formula for estimating the moment of inertia about the vertical axis (Iz) for a yacht based on its principal dimensions and displacement.

def build_initial_derivatives(pr: YachtPrincipal) -> HullDerivatives:
    U = pr.V_ref # reference speed calculated as U = V_ref
    L, T = pr.L, pr.T # Length and draft of the yacht
    rho = RHO_WATER # density of seawater

    # Linear (dimensional scale)
    Yv = -0.04 * rho * U * L * T # Sway force derivative with respect to sway velocity. Calculated as Yv = -0.04 * rho * U * L * T. -0.04 is an empirical coefficient that represents the hull's response to sway velocity.
    Yr = -0.02 * rho * U * (L**2) * T # Sway force derivative with respect to yaw rate. Calculated as Yr = -0.02 * rho * U * L^2 * T. -0.02 is an empirical coefficient that represents the hull's response to yaw rate.
    Nv = -0.02 * rho * U * (L**2) * T # Yaw moment derivative with respect to sway velocity. Calculated as Nv = -0.02 * rho * U * L^2 * T. -0.02 is an empirical coefficient that represents the hull's response to sway velocity.
    Nr = -0.04 * rho * U * (L**3) * T # Yaw moment derivative with respect to yaw rate. Calculated as Nr = -0.04 * rho * U * L^3 * T. -0.04 is an empirical coefficient that represents the hull's response to yaw rate.

    # Nonlinear damping (crossflow-inspired)
    Yvv = -0.8 * rho * T * L # Sway force derivative with respect to sway velocity squared. Calculated as Yvv = -0.8 * rho * T * L. -0.8 is an empirical coefficient that represents the nonlinear effects in the hull's lateral force response.
    Nrr = -0.25 * rho * T * (L**3) # Yaw moment derivative with respect to yaw rate squared. Calculated as Nrr = -0.25 * rho * T * L^3. -0.25 is an empirical coefficient that represents the nonlinear effects in the hull's rotational response.

    return HullDerivatives(Yv=Yv, Yr=Yr, Nv=Nv, Nr=Nr, Yvv=Yvv, Nrr=Nrr)

def build_added_mass(pr: YachtPrincipal) -> AddedMass:
    m = pr.disp_t * 1000.0
    L = pr.L
    return AddedMass(
        Xu_dot=-0.05 * m, # Surge added mass coefficient. Calculated as Xu_dot = -0.05 * m. -0.05 is an empirical coefficient that represents the additional mass in the surge direction.
        Yv_dot=-0.85 * m, # Sway added mass coefficient. Calculated as Yv_dot = -0.85 * m. -0.85 is an empirical coefficient that represents the additional mass in the sway direction.
        Nr_dot=-0.045 * m * L**2, # Yaw added mass coefficient. Calculated as Nr_dot = -0.045 * m * L^2. -0.045 is an empirical coefficient that represents the additional rotational inertia in yaw.
    )

def make_yacht_params(
    principal: YachtPrincipal = YachtPrincipal(),
    prop: Propeller = Propeller(),
    rudder: Rudder = Rudder(),
    bow: BowThruster = BowThruster(),
    tune: Dict[str, float] = None,
) -> Dict:
    """
    Returns params dict including callable force models:
      - prop_thrust(u, throttle) -> Xp [N]
      - rudder_forces(u, v, r, delta_rad, throttle) -> (XR, YR) [N]
      - hull_forces(u, v, r) -> (Xh, Yh, Nh) [N, N, N*m]
      - bow_thruster_forces(bow_cmd, xG) -> (Y_bt, N_bt) [N, N*m]
    """
    if tune is None:
        tune = {}

    m = principal.disp_t * 1000.0
    Iz = estimate_Iz(principal)

    deriv = build_initial_derivatives(principal)
    added = build_added_mass(principal)

    # Tuning multipliers (dimensionless)
    lin_hull = tune.get("lin_hull", 1.0)
    nl_hull = tune.get("nl_hull", 1.0)
    added_mul = tune.get("added", 1.0)
    thrust_mul = tune.get("thrust", 1.0)
    rudder_mul = tune.get("rudder", 1.0)
    yaw_damp_mul = tune.get("yaw_damp", 1.0)
    sway_damp_mul = tune.get("sway_damp", 1.0)
    Xuu_scale = tune.get("Xuu_scale", 1.0)  # for top-speed tuning

    deriv = HullDerivatives(
        Yv=deriv.Yv * lin_hull * sway_damp_mul, # sway damping calculated as Yv = deriv.Yv * lin_hull * sway_damp_mul
        Yr=deriv.Yr * lin_hull, # yaw damping calculated as Yr = deriv.Yr * lin_hull
        Nv=deriv.Nv * lin_hull, # sway-induced yaw damping calculated as Nv = deriv.Nv * lin_hull
        Nr=deriv.Nr * lin_hull * yaw_damp_mul,# yaw damping calculated as Nr = deriv.Nr * lin_hull * yaw_damp_mul
        Yvv=deriv.Yvv * nl_hull * sway_damp_mul,# nonlinear sway damping calculated as Yvv = deriv.Yvv * nl_hull * sway_damp_mul
        Nrr=deriv.Nrr * nl_hull * yaw_damp_mul,# nonlinear yaw damping calculated as Nrr = deriv.Nrr * nl_hull * yaw_damp_mul
    )
    added = AddedMass(
        Xu_dot=added.Xu_dot * added_mul,# surge added mass calculated as Xu_dot = added.Xu_dot * added_mul
        Yv_dot=added.Yv_dot * added_mul, # sway added mass calculated as Yv_dot = added.Yv_dot * added_mul
        Nr_dot=added.Nr_dot * added_mul, # yaw added mass calculated as Nr_dot = added.Nr_dot * added_mul
    )

    def prop_thrust(u: float, throttle: float) -> float: # computes the thrust produced by the propeller based on the vessel's forward speed (u) and the throttle setting.
        astern_eff = 0.35  # reverse is intentionally weaker than forward for gentle braking
        throttle_cmd = clamp(throttle, -1.0, 1.0)
        sign = -1.0 if throttle_cmd < 0.0 else 1.0
        throttle_mag = abs(throttle_cmd)

        u_fwd = max(0.0, u) # prevent negative inflow. Critical for J calc and reverse
        n = throttle_mag * prop.n_max_rps # propeller rotational speed [rev/s]
        if n < 1e-3: # prevent div0
            return 0.0

        Va = (1.0 - prop.w) * u_fwd # effective inflow at propeller. Calculated as Va = (1 - prop.w) * u_fwd, where prop.w is the wake fraction.
        J = Va / (n * prop.D + 1e-9) # advance ratio. Calculated as J = Va / (n * prop.D), where prop.D is the propeller diameter. Advane ratio is a dimensionless number that characterizes the operating condition of the propeller.

        KT = prop.KT_a0 + prop.KT_a1 * J + prop.KT_a2 * (J**2) # thrust coefficient polynomial. Calculated using a quadratic polynomial based on the advance ratio J.
        KT = max(0.0, KT) # prevent negative thrust at high J

        T = RHO_WATER * (n**2) * (prop.D**4) * KT # thrust [N]. Calculated as T = rho * n^2 * D^4 * KT, where rho is the density of water.
        Xp = (1.0 - prop.t_deduction) * T # effective thrust accounting for thrust deduction factor.
        if sign < 0.0:
            Xp *= -astern_eff
        return thrust_mul * Xp # apply tuning

    def rudder_forces(u: float, v: float, r: float, delta_rad: float, throttle: float) -> Tuple[float, float]:
        """
        Lift/drag rudder. Outputs (XR, YR) in body axes (+X forward, +Z sway).
        Includes:
          - local lateral inflow v_R = v + xR*r
          - propwash augmentation
          - small deadband to guarantee "no rudder => no force" at symmetric condition
        """
        # Deadband on commanded rudder. This deadband helps to eliminate small fluctuations in the rudder angle that may occur due to noise or minor adjustments, ensuring that the rudder only produces forces when the angle exceeds a certain threshold.
        deadband = math.radians(rudder.deadband_deg) # this converts the deadband angle from degrees to radians.
        if abs(delta_rad) < deadband:
            delta_rad = 0.0 #to enforce zero rudder angle within the deadband range.

        v_R = v + rudder.xR * r # local lateral inflow at rudder. Calculated as v_R = v + xR * r, where xR is the distance from the midship to the rudder along the longitudinal axis.

        # Guarantee symmetry at straight + no lateral inflow:
        # If delta=0 and v_R is tiny, return exactly zero forces (prevents drift).
        if delta_rad == 0.0 and abs(v_R) < 1e-4 and abs(r) < 1e-4: #delta is the rudder angle in radians.
            return 0.0, 0.0

        u_fwd = max(0.5, u)  # prevent inflow collapse
        Xp_raw = prop_thrust(u, throttle) / max(1.0 - prop.t_deduction, 1e-6) # raw prop thrust before deduction

        A_disk = math.pi * (prop.D**2) / 4.0 # propeller disk area
        v_jet = math.sqrt(max(0.0, Xp_raw) / (RHO_WATER * A_disk + 1e-9)) # approximate jet velocity behind propeller
        U_axial = (1.0 - prop.w) * u_fwd + rudder.k_propwash * v_jet # axial inflow at rudder including propwash

        beta_R = math.atan2(v_R, U_axial) # rudder inflow angle
        alpha = delta_rad - beta_R # rudder angle of attack

        CL = rudder.CL_alpha * alpha # lift coefficient (calculated as CL = rudder.CL_alpha * alpha, where rudder.CL_alpha is the lift curve slope of the rudder.)
        CD = rudder.CD0 + rudder.k_induced * (CL**2) # drag coefficient calculated as CD = rudder.CD0 + rudder.k_induced * (CL^2). This formula accounts for both the zero-lift drag and the induced drag due to lift.

        U_R = math.sqrt(U_axial**2 + v_R**2) # total inflow speed at rudder
        q = 0.5 * RHO_WATER * U_R**2 # dynamic pressure at rudder

        Lf = q * rudder.A * CL # rudder lift force
        Df = q * rudder.A * CD # rudder drag force

        c = U_axial / (U_R + 1e-9) # cosine/sine of inflow angle
        s = v_R / (U_R + 1e-9) # sine of inflow angle

        # Drag along inflow (opposes inflow)
        Xd = -Df * c # surge component of drag force calculated as Xd = -Df * c, where c is the cosine of the inflow angle.
        Yd = -Df * s # sway component of drag force calculated as Yd = -Df * s, where s is the sine of the inflow angle.

        # Lift perpendicular to inflow (left-normal [-s, c])
        Xl = Lf * (-s) # surge component of lift force calculated as Xl = Lf * (-s), where s is the sine of the inflow angle.
        Yl = Lf * (c) # sway component of lift force calculated as Yl = Lf * c, where c is the cosine of the inflow angle.

        XR = (Xd + Xl) * rudder.k_eff * rudder_mul # total surge force from rudder calculated as XR = (Xd + Xl) * rudder.k_eff * rudder_mul
        YR = (Yd + Yl) * rudder.k_eff * rudder_mul # total sway force from rudder calculated as YR = (Yd + Yl) * rudder.k_eff * rudder_mul
        return XR, YR

    def hull_forces(u: float, v: float, r: float) -> Tuple[float, float, float]:
        """
        Hull forces:
          - Surge resistance ~ u|u|
          - Linear + nonlinear sway/yaw damping
        """
        # Simple quadratic surge resistance (tune with Xuu_scale)
        Xuu = Xuu_scale * (0.5 * RHO_WATER * principal.B * principal.T * 0.9) # surge resistance coefficient calculated as Xuu = Xuu_scale * (0.5 * RHO_WATER * principal.B * principal.T * 0.9). This formula estimates the surge resistance based on the yacht's beam (B) and draft (T), along with the water density.
        Xh = -Xuu * u * abs(u) # surge force calculated as Xh = -Xuu * u * abs(u), representing the resistance opposing the forward motion of the yacht.

        Yh = deriv.Yv * v + deriv.Yr * r + deriv.Yvv * v * abs(v) # sway force calculated as Yh = deriv.Yv * v + deriv.Yr * r + deriv.Yvv * v * abs(v). This formula combines linear and nonlinear components to represent the lateral forces acting on the hull.
        Nh = deriv.Nv * v + deriv.Nr * r + deriv.Nrr * r * abs(r) # yaw moment calculated as Nh = deriv.Nv * v + deriv.Nr * r + deriv.Nrr * r * abs(r). This formula combines linear and nonlinear components to represent the rotational forces acting on the hull.
        return Xh, Yh, Nh
    def bow_thruster_forces(bow_cmd: float, xG: float) -> Tuple[float, float]:
        """Bow thruster forces/moment in body axes (u:+X, v:+Z, r:+Y).

        bow_cmd in [-1, 1]. Positive command => force in body +Z direction.
        Returns:
          Y_bt [N] and N_bt [N*m]
        """
        bow_cmd = float(bow_cmd)
        if abs(bow_cmd) < bow.deadband:
            bow_cmd = 0.0
        bow_cmd = clamp(bow_cmd, -1.0, 1.0)

        Y_bt = bow_cmd * bow.Y_max
        N_bt = (bow.xBT - xG) * Y_bt
        return Y_bt, N_bt



    return {
        "L": principal.L,
        "B": principal.B,
        "T": principal.T,
        "m": m,
        "Iz": Iz,
        "xG": principal.xG,

        "Xu_dot": added.Xu_dot,
        "Yv_dot": added.Yv_dot,
        "Nr_dot": added.Nr_dot,

        "prop": prop,
        "rudder": rudder,
        "bow_thruster": bow,
        "tune": tune,

        "prop_thrust": prop_thrust,
        "rudder_forces": rudder_forces,
        "hull_forces": hull_forces,
        "bow_thruster_forces": bow_thruster_forces,
    }