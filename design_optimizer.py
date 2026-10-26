"""NeedleTail unified propulsion/control design optimizer (concept screening).

A 3-axis bounded control allocator compares tractor4, pusher4, coaxial8
combined with 0/2/4 independent flaps. Results do NOT predict top speed:
until powered aerodynamics/propeller maps are calibrated, use this only for
relative control/mass/drag design screening.

Run: uv run --locked --extra aero python design_optimizer.py
     uv run --locked --extra aero python design_optimizer.py --settings design_optimizer_settings.json
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass, field, replace
from itertools import product
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

from design_configuration import (DEFAULT_MASS_ASSUMPTIONS_G, Design, FLAP_MODES, TOPOLOGIES, flap_segments,
                                  mass_breakdown_g, rotor_stations, configuration_name)


MASS_SCENARIO_KEYS = {
    "motor_mass_g", "esc_mass_g", "prop_mass_g", "rear_mount_mass_g", "servo_mass_g",
}


@dataclass(frozen=True)
class Settings:
    """All numbers are explicit trial assumptions, NOT certified ratings."""
    speed_mps: float = 30.0
    rho_kg_m3: float = 1.225
    per_rotor_thrust_N: float = 1.0
    reaction_torque_per_thrust_m: float = 0.004
    coaxial_pair_efficiency: float = .85
    installed_pusher_efficiency: float = 1.0
    flap_lift_slope_per_rad: float = 1.9
    flap_gap_cd: float = .006
    flap_drag_k: float = .11
    required_roll_Nm: float = .003
    required_pitch_Nm: float = .006
    required_yaw_Nm: float = .006
    servo_mass_g: float = DEFAULT_MASS_ASSUMPTIONS_G["servo_mass_g"]
    motor_mass_g: float = DEFAULT_MASS_ASSUMPTIONS_G["motor_mass_g"]
    esc_mass_g: float = DEFAULT_MASS_ASSUMPTIONS_G["esc_mass_g"]
    prop_mass_g: float = DEFAULT_MASS_ASSUMPTIONS_G["prop_mass_g"]
    rear_mount_mass_g: float = DEFAULT_MASS_ASSUMPTIONS_G["rear_mount_mass_g"]
    # Mass expressed as a thrust-equivalent score penalty at the studied
    # speed; not a physically solved change in flight trim.
    weight_penalty_fraction: float = .08
    require_modelled_control: bool = True
    geometry_grid_mm: dict[str, list[float]] = field(default_factory=dict)
    # Named scenarios override only the listed mass assumptions. Every
    # omitted component inherits the values above.
    mass_scenarios: dict[str, dict[str, float]] = field(default_factory=dict)

    def __post_init__(self):
        for key,value in asdict(self).items():
            if isinstance(value,(float,int)) and (not math.isfinite(value) or value<0):
                raise ValueError(f"Nonfinite/negative setting: {key}")
        for key in MASS_SCENARIO_KEYS:
            value=getattr(self,key)
            if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<0:
                raise ValueError(f"Invalid mass assumption: {key}")
        if not .5<=self.coaxial_pair_efficiency<=1.0:raise ValueError("Coaxial ratio out of bounds")
        if not .5<=self.installed_pusher_efficiency<=1.5:raise ValueError("Pusher ratio out of bounds")
        allowed = {"wing_root_chord","wing_tip_chord","horizontal_motor_offset",
                   "vertical_motor_offset","fuselage_length"}
        if not set(self.geometry_grid_mm).issubset(allowed):
            raise ValueError("Unknown geometry sweep parameter")
        if any(not isinstance(v,list) or not v or
               any(not isinstance(x,(float,int)) or not math.isfinite(x) or x<=0 for x in v)
               for v in self.geometry_grid_mm.values()):
            raise ValueError("Geometry grids require nonempty lists of positive dimensions")
        if not isinstance(self.mass_scenarios, dict):
            raise ValueError("Mass scenarios must be a mapping of names to mass overrides")
        for name, overrides in self.mass_scenarios.items():
            if not isinstance(name, str) or not name.strip() or not isinstance(overrides, dict):
                raise ValueError("Mass scenarios require nonempty names and override mappings")
            if not set(overrides).issubset(MASS_SCENARIO_KEYS):
                raise ValueError(f"Unknown mass scenario setting: {set(overrides) - MASS_SCENARIO_KEYS}")
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or
                   not math.isfinite(value) or value < 0 for value in overrides.values()):
                raise ValueError(f"Invalid mass override in scenario {name!r}")
        if not (0<self.per_rotor_thrust_N and 0<self.rho_kg_m3):
            raise ValueError("Unphysical rotor limit or density")


def _rotor_matrix(p, d, settings):
    rotors=rotor_stations(p,d)
    positions=np.asarray([r.xyz_mm for r in rotors],float)/1000
    mat=np.zeros((3,len(rotors)))
    # All axes are CAD +X. Relative CG is taken at the CAD origin until
    # true mass properties and motor thrust lines are established.
    for i,r in enumerate(rotors):
        mat[:,i]=np.cross(positions[i], [1.,0.,0.]) + [
            r.spin_sign*settings.reaction_torque_per_thrust_m,0.,0.]
    bounds=[]
    for r in rotors:
        ratio=(settings.installed_pusher_efficiency if d.topology=="pusher4" else
               (2*settings.coaxial_pair_efficiency-1 if d.topology=="coaxial8" and r.side=="rear" else 1.))
        bounds.append((0.,settings.per_rotor_thrust_N*ratio))
    return mat,bounds


def _flap_matrix(p,d,settings):
    flaps=flap_segments(p,d)
    mat=np.zeros((3,len(flaps)))
    areas=[]
    for i,(name,axis,a,b) in enumerate(flaps):
        halfspan=p.wing_span/2 if axis=="horizontal" else p.support_span/2
        mean_span=abs((a+b)/2)
        chord=(p.support_chord if axis=="vertical" else
               p.wing_root_chord+(p.wing_tip_chord-p.wing_root_chord)*mean_span/halfspan)
        area=(b-a)*chord*1.e-6
        # CAD +X forward, trailing edge -X. Aero center is approximated
        # halfway between hinge and TE, with fixed flap effectiveness.
        x=(.5-(1+d.hinge_fraction)/2)*chord/1000
        q=.5*settings.rho_kg_m3*settings.speed_mps**2
        hinge_gain=((1-d.hinge_fraction)/.28)**.7
        f=q*area*settings.flap_lift_slope_per_rad*hinge_gain
        normal=np.asarray((0.,0.,1.) if axis=="horizontal" else (0.,1.,0.))
        location=(x,(a+b)/2000.,0.) if axis=="horizontal" else (x,0.,(a+b)/2000.)
        mat[:,i]=np.cross(location,normal*f)
        areas.append(area)
    return mat,flaps,areas


def _asb_moments_to_cad(moments):
    """Convert 3xN AeroSandbox moments to CAD body-axis moments.

    Project +X points forward while AeroSandbox +X points aft. Moment is an
    axial vector, so reflecting X maps (Mx, My, Mz) to (Mx, -My, -Mz).
    """
    values=np.asarray(moments,dtype=float)
    if values.ndim!=2 or values.shape[0]!=3 or not np.isfinite(values).all():
        raise ValueError("AeroSandbox moments must be a finite 3 x N matrix")
    return np.diag([1.,-1.,-1.]) @ values


def evaluate(p, design:Design, settings:Settings, derivatives=None):
    """Maximize available common rotor thrust subject to pitch/roll/yaw torque.

    If provided, derivatives MUST be a 3 x N matrix in CAD body axes, Nm/rad.
    They can come from verified aero measurements. Otherwise a transparent
    low-fidelity panel proxy is used when require_modelled_control=True.
    """
    Arot,motor_bounds=_rotor_matrix(p,design,settings)
    Aflap,panels,areas=_flap_matrix(p,design,settings)
    if derivatives is not None:
        Aflap=np.asarray(derivatives,float)
        if Aflap.shape!=(3,len(panels)) or not np.isfinite(Aflap).all():
            raise ValueError("Control-derivative matrix has invalid dimensions/values")
    elif not settings.require_modelled_control:
        Aflap=np.zeros_like(Aflap)
    target=np.asarray((settings.required_roll_Nm,
                       settings.required_pitch_Nm,settings.required_yaw_Nm))
    limit=math.radians(design.flap_limit_deg)
    bounds=motor_bounds+[(-limit,limit)]*len(panels)
    A=np.column_stack((Arot,Aflap))
    goal=np.asarray([-1.]*len(motor_bounds)+[0.]*len(panels))
    res=linprog(goal,A_eq=A,b_eq=target,bounds=bounds,method="highs")
    mass_breakdown=mass_breakdown_g(
        design,
        motor_g=settings.motor_mass_g,
        esc_g=settings.esc_mass_g,
        prop_g=settings.prop_mass_g,
        rear_mount_g=settings.rear_mount_mass_g,
        flap_actuator_g=settings.servo_mass_g,
    )
    mass=mass_breakdown["total_added_g"]
    base=dict(topology=design.topology,flaps=design.flaps,
              hinge_fraction=design.hinge_fraction,
              flap_start_fraction=design.flap_start_fraction,
              flap_end_fraction=design.flap_end_fraction,
              flap_limit_deg=design.flap_limit_deg,
              coaxial_separation_mm=design.coaxial_separation_mm,
              n_motors=len(motor_bounds),n_flaps=len(panels),
              added_mass_g=mass,
              mass_breakdown_g=mass_breakdown,
              mass_assumptions_g={
                  "motor_mass_g":settings.motor_mass_g,
                  "esc_mass_g":settings.esc_mass_g,
                  "prop_mass_g":settings.prop_mass_g,
                  "rear_mount_mass_g":settings.rear_mount_mass_g,
                  "servo_mass_g":settings.servo_mass_g,
              },
              speed_mps=settings.speed_mps,
              control_model=("supplied_derivatives_experimental" if derivatives is not None else
                             "assumed_proxy" if settings.require_modelled_control else "zero"),
              cad_validated=False, minimum_cad_clearance_mm=None, cad_error="",
              propeller_map_validated=False)
    if not res.success:
        return dict(**base,feasible=False, reason=str(res.message),
                    score_N=None,available_collective_N=None,thrust_sacrificed_N=None)
    t=res.x[:len(motor_bounds)]
    delta=res.x[len(motor_bounds):]
    q=.5*settings.rho_kg_m3*settings.speed_mps**2
    extra_drag=sum(q*area*(settings.flap_gap_cd+settings.flap_drag_k*angle**2)
                   for area,angle in zip(areas,delta))
    # optimize collective thrust with a drag and mass penalty. This is a
    # screening score, deliberately not flight speed or electrical power.
    collective=float(sum(t))
    score=(collective-extra_drag-
           mass*1.e-3*9.80665*settings.weight_penalty_fraction)
    return dict(**base,feasible=True, reason="",
        score_N=score,available_collective_N=collective,
        thrust_sacrificed_N=float(sum(b[1] for b in motor_bounds)-collective),
        flap_incremental_drag_N=float(extra_drag),
        assumed_actuator_deflections_deg=[float(v) for v in np.degrees(delta)],
        moment_residual_Nm=float(np.max(np.abs(A@res.x-target))),
        flap_roll_limit_Nm=float(np.sum(np.abs(Aflap[0])*limit)),
        flap_pitch_limit_Nm=float(np.sum(np.abs(Aflap[1])*limit)),
        flap_yaw_limit_Nm=float(np.sum(np.abs(Aflap[2])*limit)))


def candidate_designs():
    for topology,flaps in product(TOPOLOGIES,FLAP_MODES):
        if flaps=="none":
            yield Design(topology=topology)
        else:
            for hinge,start,limit in product((.65,.72,.80),(.18,.35),(15.,25.)):
                yield Design(topology=topology,flaps=flaps,hinge_fraction=hinge,
                             flap_start_fraction=start,flap_limit_deg=limit)


def airframes_from_grid(p, settings):
    """Generate geometric candidates directly from AirframeParameters."""
    if not settings.geometry_grid_mm:
        yield p, {}
        return
    if not hasattr(p, "__dataclass_fields__"):
        raise TypeError("Geometry grid requires the real AirframeParameters dataclass")
    keys=tuple(sorted(settings.geometry_grid_mm))
    for values in product(*(settings.geometry_grid_mm[k] for k in keys)):
        updated=dict(zip(keys,values))
        candidate=replace(p,**updated)
        if candidate.wing_root_chord<=candidate.wing_tip_chord:
            continue
        try:
            candidate.validate_arm_pod_fit()
            candidate.validate_motor_prop_interface()
        except ValueError:
            continue
        yield candidate,updated


def optimize(p, settings, derivative_provider=None):
    """Search all nine mode families and optional CAD geometry dimensions.

    A validated derivative_provider(params, design) -> 3xN (Nm/radian)
    can replace the illustrative control proxy in each candidate.
    """
    results=[]
    scenario_overrides=list(settings.mass_scenarios.items()) or [("default",{})]
    scenarios=[
        (name,replace(settings,**overrides,mass_scenarios={}))
        for name,overrides in scenario_overrides
    ]
    rankings={name:[] for name,_ in scenarios}
    for params, changed_mm in airframes_from_grid(p,settings):
        for d in candidate_designs():
            derivatives=derivative_provider(params,d) if derivative_provider else None
            for scenario_name, scenario_settings in scenarios:
                result=evaluate(params,d,scenario_settings,derivatives=derivatives)
                result["mass_scenario"]=scenario_name
                result["scenario_rank"]=0
                result["geometry_parameters_mm"]=changed_mm
                rankings[scenario_name].append(result)
    for scenario_rows in rankings.values():
        scenario_rows.sort(key=lambda row:(row["feasible"],
                           row["score_N"] if row["score_N"] is not None else -math.inf),
                           reverse=True)
        for rank,row in enumerate(scenario_rows,1):
            row["scenario_rank"]=rank
        results.extend(scenario_rows)
    return results


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--settings",type=Path)
    parser.add_argument("--output-dir",type=Path,default=Path("tmp/design-optimizer"))
    parser.add_argument("--experimental-aero-derivatives",action="store_true",
                        help="Slow unpowered AeroSandbox flap derivatives in CAD body axes")
    parser.add_argument("--check-best-cad",action="store_true",
                        help="Expensive exact CAD checks; report failure without hiding it")
    args=parser.parse_args(argv)
    from airframe_geometry import PARAMETERS
    settings=Settings(**json.loads(args.settings.read_text())) if args.settings else Settings()
    provider=None
    if args.experimental_aero_derivatives:
        from design_aero import flap_derivatives
        from design_configuration import flap_segments
        def provider(params, design):
            if design.flaps == "none":
                return np.zeros((3,0))
            names, moments_asb = flap_derivatives(
                params,design,speed_mps=max(settings.speed_mps,.1))
            wanted=[seg[0] for seg in flap_segments(params,design)]
            if names != wanted:
                raise ValueError(f"Flap order mismatch: {names} != {wanted}")
            # CAD +X forward, ASB +X aft: pseudovector moment transform.
            # Control polarity still requires physical verification.
            return _asb_moments_to_cad(moments_asb)
    results=optimize(PARAMETERS,settings,derivative_provider=provider)
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.check_best_cad:
        from design_cad import validate_variant
        cad_cache={}
        for row in results:
            if row["scenario_rank"] > 9:
                continue
            cad_key=(
                row["topology"],row["flaps"],row["hinge_fraction"],
                row["flap_start_fraction"],row["flap_end_fraction"],
                row["flap_limit_deg"],row["coaxial_separation_mm"],
                tuple(sorted(row["geometry_parameters_mm"].items())),
            )
            if cad_key in cad_cache:
                row.update(cad_cache[cad_key])
                continue
            choice=Design(**{k:row[k] for k in (
                "topology","flaps","hinge_fraction","flap_start_fraction",
                "flap_end_fraction","flap_limit_deg","coaxial_separation_mm")})
            try:
                airframe=replace(PARAMETERS,**row["geometry_parameters_mm"])
                clear=validate_variant(airframe,choice,airframe.propeller_clearance_margin)
                validation={"cad_validated":True,
                            "minimum_cad_clearance_mm":min(clear.values()),
                            "cad_error":""}
            except (ValueError,RuntimeError) as ex:
                validation={"cad_validated":False,"cad_error":str(ex)}
            cad_cache[cad_key]=validation
            row.update(validation)
    (args.output_dir/"designs.json").write_text(json.dumps(
        {"assumptions":asdict(settings),"candidates":results},indent=2))
    with (args.output_dir/"designs.csv").open("w",newline="") as f:
        columns=[k for k in results[0] if not isinstance(results[0][k],list)]
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction="ignore")
        writer.writeheader()
        for row in results:
            writer.writerow({k:json.dumps(v) if isinstance(v,dict) else v for k,v in row.items()})
    print(f"Compared {len(results)} configurations; results at {args.output_dir}")
    leaders={row["mass_scenario"]:row for row in results if row["scenario_rank"]==1}
    print("Highest screening score by mass scenario (not a maximum-speed prediction):")
    print(json.dumps(leaders,indent=2))


if __name__=="__main__":
    main()
