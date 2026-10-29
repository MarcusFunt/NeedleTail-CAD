"""Optional adapter for the checksum-pinned qprop.c portable Python binding."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import time
from typing import Mapping

from propulsion.models import PropellerResult


QPROPC_REVISION = "2603@caacc6623a3b3596c9d59ff361ed647db40508a8"
QPROPC_ARCHIVE_SHA256 = "aa9e79fd9cc4cc6051bf5d134e8436edfb3c52e7f9e4fe27f2788962a071ae0a"
SUPPORTED_LIBRARY_NAMES = (
    "qprop-lib-windows-x64.dll",
    "qprop-lib-linux-x64.so",
    "qprop-lib-macos-arm64.dylib",
)


def frozen_bemt_common_polar_parameters() -> dict[str, float]:
    """Analytic qprop polar matching frozen BEMT's CL/CD curves exactly."""
    return {
        "CL0": 0.0,
        "CL_a": 2.0 * math.pi,
        "CLmin": -1.2,
        "CLmax": 1.2,
        "CD0": 0.025,
        "CD2u": 0.055,
        "CD2l": 0.055,
        "CLCD0": 0.0,
        "REref": 70000.0,
        "REexp": 0.0,
    }


def propeller_is_out_of_scope(propeller_id: str, out_of_scope_propellers: tuple[str, ...]) -> bool:
    """Whether validation evidence excludes a propeller from the model scope."""
    return propeller_id in out_of_scope_propellers


def polar_sample_in_coverage(
    polar_limits: tuple[tuple[float, float, float], ...],
    reynolds: float,
    alpha_degrees: float,
) -> bool:
    """Return whether an XFOIL sample is bracketed in Re and shared alpha range."""
    if not polar_limits or not math.isfinite(reynolds) or not math.isfinite(alpha_degrees):
        return False
    ordered = sorted(polar_limits)
    for polar_re, alpha_min, alpha_max in ordered:
        if math.isclose(reynolds, polar_re, rel_tol=1e-9, abs_tol=1e-6):
            return alpha_min <= alpha_degrees <= alpha_max
    lower = None
    upper = None
    for limit in ordered:
        if limit[0] < reynolds:
            lower = limit
        elif limit[0] > reynolds:
            upper = limit
            break
    if lower is None or upper is None:
        return False
    alpha_min = max(lower[1], upper[1])
    alpha_max = min(lower[2], upper[2])
    return alpha_min <= alpha_degrees <= alpha_max


def _load_qprop_module(portable_dir: Path):
    wrapper = portable_dir / "qprop.py"
    if not wrapper.is_file():
        raise FileNotFoundError(f"qprop.py not found under {portable_dir}")
    spec = importlib.util.spec_from_file_location("_needletail_qpropc", wrapper)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load qprop.py from {wrapper}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class QPropCBackend:
    """One shared airfoil polar set and a common input contract for qprop.c."""

    name = "qpropc"

    def __init__(
        self,
        portable_dir: str | Path,
        polar_dir: str | Path | None = None,
        *,
        polar_mode: str = "xfoil",
        polar_source_description: str | None = None,
        scenario_name: str | None = None,
        native_geometry_paths: Mapping[str, str | Path] | None = None,
        out_of_scope_propellers: tuple[str, ...] = (),
        rho_kg_m3: float = 1.225,
        dynamic_viscosity_pa_s: float = 1.81e-5,
        speed_of_sound_mps: float = 0.0,
    ):
        start = time.perf_counter()
        for parameter_name, parameter_value in (
            ("rho_kg_m3", rho_kg_m3),
            ("dynamic_viscosity_pa_s", dynamic_viscosity_pa_s),
            ("speed_of_sound_mps", speed_of_sound_mps),
        ):
            if not math.isfinite(parameter_value):
                raise ValueError(f"{parameter_name} must be finite")
        if rho_kg_m3 <= 0 or dynamic_viscosity_pa_s <= 0 or speed_of_sound_mps < 0:
            raise ValueError("density and dynamic viscosity must be positive; speed of sound must be nonnegative")
        self.portable_dir = Path(portable_dir).resolve()
        manifest_path = self.portable_dir / "release.json"
        if not manifest_path.is_file():
            raise ValueError(
                f"unverified qprop.c portable library; run validation/qpropc/install_qpropc.py --dest <path>"
            )
        release = json.loads(manifest_path.read_text(encoding="utf-8"))
        if (release.get("release_tag") != "2603"
                or release.get("source_commit") != QPROPC_REVISION.split("@", 1)[1]
                or release.get("archive_sha256") != QPROPC_ARCHIVE_SHA256
                or release.get("license") != "MIT"):
            raise ValueError("qprop.c release manifest does not match the pinned MIT release")
        for name, expected in release.get("files", {}).items():
            file_path = self.portable_dir / name
            if not file_path.is_file() or hashlib.sha256(file_path.read_bytes()).hexdigest() != expected:
                raise ValueError(f"qprop.c installed file failed release hash verification: {name}")
        if not any((self.portable_dir / name).is_file() for name in SUPPORTED_LIBRARY_NAMES):
            raise FileNotFoundError(
                f"qprop.c native library is missing; expected one of {', '.join(SUPPORTED_LIBRARY_NAMES)}"
            )
        if polar_mode not in {"xfoil", "common_analytic"}:
            raise ValueError("polar_mode must be 'xfoil' or 'common_analytic'")
        self.polar_mode = polar_mode
        self.polar_dir = Path(polar_dir).resolve() if polar_dir is not None else None
        if polar_mode == "xfoil":
            if self.polar_dir is None or not self.polar_dir.is_dir():
                raise FileNotFoundError(f"XFOIL polar directory not found: {self.polar_dir}")
            self.polar_files = tuple(sorted(self.polar_dir.glob("*.txt")))
            if not self.polar_files:
                raise ValueError(f"no XFOIL .txt polar files found in {self.polar_dir}")
            digest = hashlib.sha256()
            for path in self.polar_files:
                digest.update(path.name.encode("utf-8"))
                digest.update(b"\0")
                digest.update(path.read_bytes())
            self.polar_sha256 = digest.hexdigest()
            self.polar_coverage = _polar_coverage(self.polar_files)
            self.polar_limits = _polar_file_limits(self.polar_files)
            self.polar_source_description = polar_source_description or f"XFOIL files from {self.polar_dir}"
        else:
            if polar_dir is not None:
                raise ValueError("polar_dir must be omitted when polar_mode='common_analytic'")
            parameters = frozen_bemt_common_polar_parameters()
            self.polar_files = ()
            self.polar_sha256 = hashlib.sha256(
                json.dumps(parameters, sort_keys=True).encode("utf-8")
            ).hexdigest()
            self.polar_coverage = {"type": "common analytic polar", "reynolds_exponent": 0.0}
            self.polar_limits = ()
            self.polar_source_description = (
                polar_source_description or "controlled common analytic polar matching frozen BEMT CL/CD"
            )
        self.name = scenario_name or f"qpropc_{self.polar_mode}"
        self.rho_kg_m3 = rho_kg_m3
        self.dynamic_viscosity_pa_s = dynamic_viscosity_pa_s
        self.speed_of_sound_mps = speed_of_sound_mps
        self.native_geometry_paths = {
            propeller_id: Path(path).resolve()
            for propeller_id, path in (native_geometry_paths or {}).items()
        }
        self.out_of_scope_propellers = tuple(out_of_scope_propellers)
        for propeller_id, path in self.native_geometry_paths.items():
            if not path.is_file():
                raise FileNotFoundError(f"native APC geometry for {propeller_id} not found: {path}")
        self.native_geometry_sha256_by_propeller = {
            propeller_id: hashlib.sha256(path.read_bytes()).hexdigest()
            for propeller_id, path in self.native_geometry_paths.items()
        }
        try:
            self.module = _load_qprop_module(self.portable_dir)
            if self.polar_mode == "xfoil":
                self.airfoil = self.module.import_xfoil_polars([str(path) for path in self.polar_files])
            else:
                self.airfoil = self.module.analytic_polar_curves(
                    *frozen_bemt_common_polar_parameters().values()
                )
        except Exception as exc:
            raise RuntimeError(f"qprop.c binding could not load the pinned native library or polars: {exc}") from exc
        self.revision = QPROPC_REVISION
        self.input_configuration = {
            "qpropc_revision": self.revision,
            "portable_archive_sha256": release["archive_sha256"],
            "portable_file_sha256": release["files"],
            "polar_mode": self.polar_mode,
            "polar_sha256": self.polar_sha256,
            "rho_kg_m3": self.rho_kg_m3,
            "dynamic_viscosity_pa_s": self.dynamic_viscosity_pa_s,
            "speed_of_sound_mps": self.speed_of_sound_mps,
        }
        self.configuration = {
            **self.input_configuration,
            "polar_source_description": self.polar_source_description,
            "native_geometry_sha256_by_propeller": self.native_geometry_sha256_by_propeller,
        }
        self.initialization_ms = (time.perf_counter() - start) * 1000
        self._closed = False

    def predict(self, case):
        if self._closed:
            raise RuntimeError("qprop.c backend is closed")
        case.geometry.validate_for_prediction()
        native_geometry = self.native_geometry_paths.get(case.propeller_id)
        native_sha = None
        if native_geometry is not None:
            native_sha = hashlib.sha256(native_geometry.read_bytes()).hexdigest()
            if native_sha != self.native_geometry_sha256_by_propeller[case.propeller_id]:
                raise ValueError(f"native APC geometry changed after backend initialization: {case.propeller_id}")
            rotor = self.module.import_rotor_geometry_apc(str(native_geometry), self.airfoil)
        else:
            rotor = self.module.create_rotor(
                case.geometry.diameter_m,
                case.geometry.blade_count,
                len(case.geometry.stations),
                [self.module.create_section(
                    station.chord_m,
                    station.twist_rad,
                    station.radius_m,
                    self.airfoil,
                ) for station in case.geometry.stations],
            )
        input_hash = hashlib.sha256(json.dumps(
            {
                "case_input_sha256": case.input_sha256,
                "backend_inputs": self.input_configuration,
                "native_geometry_sha256": native_sha,
            },
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")).hexdigest()
        omega = case.rpm * 2.0 * 3.141592653589793 / 60.0
        freestream = case.advance_ratio * (case.rpm / 60.0) * case.geometry.diameter_m
        start = time.perf_counter()
        performance = None
        try:
            performance = self.module.qprop(
                rotor,
                freestream,
                omega,
                rho=self.rho_kg_m3,
                mu=self.dynamic_viscosity_pa_s,
                a=self.speed_of_sound_mps,
            )
            nelems = int(performance.nelems)
            thrust = float(performance.T)
            torque = float(performance.Q)
            ct = float(performance.CT)
            cp = float(performance.CP)
            advance_ratio = float(performance.J)
            residual_max = max((abs(float(performance.residuals[i])) for i in range(nelems)), default=0.0)
            polar_covered = False
            if self.polar_mode == "xfoil":
                operating_samples = _operating_samples(
                    rotor,
                    performance,
                    self.rho_kg_m3,
                    self.dynamic_viscosity_pa_s,
                )
                polar_covered = bool(operating_samples) and all(
                    polar_sample_in_coverage(self.polar_limits, reynolds, alpha_degrees)
                    for reynolds, alpha_degrees in operating_samples
                )
        finally:
            if performance is not None:
                self.module.free_rotor_performance(performance)
            # Native-imported APC rotors own C-allocated memory. Rotors created
            # by qprop.py's create_rotor instead own a ctypes/Python array and
            # must be released by Python's object lifetime, not free_rotor.
            if native_geometry is not None:
                self.module.free_rotor(rotor)
        elapsed_ms = (time.perf_counter() - start) * 1000
        values = (thrust, torque, ct, cp, advance_ratio, residual_max)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("qprop.c returned a nonfinite load, coefficient, or residual")
        converged = residual_max <= 1e-6
        exact_geometry = (
            native_geometry is not None
            or case.geometry.geometry_provenance.value in {"measured", "manufacturer"}
        )
        propulsive = ct > 0 and cp > 0
        out_of_scope = propeller_is_out_of_scope(case.propeller_id, self.out_of_scope_propellers)
        warnings = ["qprop_2603", f"polar_mode_{self.polar_mode}", "airfoil_section_fidelity_must_be_checked"]
        if not exact_geometry:
            warnings.append("approximate_or_digitized_geometry")
        if not propulsive:
            warnings.append("nonpropulsive_or_windmilling_regime")
        if out_of_scope:
            warnings.append("propeller_excluded_from_validation_scope")
        if self.polar_mode == "common_analytic":
            warnings.append("common_analytic_control_polar_is_not_empirical_validity_evidence")
        elif not polar_covered:
            warnings.append("local_reynolds_or_angle_outside_xfoil_polar_coverage")
        if not converged:
            warnings.append(f"residual_max={residual_max:.6g}")
        return PropellerResult(
            thrust_n=thrust,
            torque_nm=torque,
            shaft_power_w=torque * omega,
            ct=ct,
            cp=cp,
            advance_ratio=advance_ratio,
            efficiency=(advance_ratio * ct / cp if advance_ratio > 0 and ct > 0 and cp > 0 else None),
            converged=converged,
            within_validity_range=exact_geometry and propulsive and not out_of_scope and polar_covered,
            warnings=tuple(warnings),
            solver_name=self.name,
            solver_revision=self.revision,
            input_sha256=input_hash,
            elapsed_ms=elapsed_ms,
            geometry_valid=True,
            convergence_evidence=(f"nelems={nelems}", f"max_abs_residual={residual_max:.9g}"),
            model_scope=(
                f"qprop.c source revision {QPROPC_REVISION}; {self.polar_source_description}; "
                "slender-rotor vortex/BEM model, no externally induced tangential velocity"
            ),
        )

    def close(self) -> None:
        if not self._closed:
            self.module.free_airfoil(self.airfoil)
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


def _polar_coverage(paths: tuple[Path, ...]) -> dict[str, float | int | str | None]:
    import re

    reynolds, alpha_degrees = [], []
    re_pattern = re.compile(r"\bRe\s*=\s*([0-9.eE+-]+)")
    for path in paths:
        for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
            match = re_pattern.search(line)
            if match:
                try:
                    reynolds.append(float(match.group(1)) * 1e6)
                except ValueError:
                    pass
            fields = line.split()
            if len(fields) >= 3:
                try:
                    alpha_degrees.append(float(fields[0]))
                    float(fields[1])
                    float(fields[2])
                except ValueError:
                    continue
    return {
        "file_count": len(paths),
        "re_min": min(reynolds) if reynolds else None,
        "re_max": max(reynolds) if reynolds else None,
        "re_unit": "dimensionless Reynolds number",
        "alpha_min_deg": min(alpha_degrees) if alpha_degrees else None,
        "alpha_max_deg": max(alpha_degrees) if alpha_degrees else None,
        "xfoil_nonconvergence_evidence": "not available in polar text files; omitted cases cannot be distinguished",
    }


def _polar_file_limits(paths: tuple[Path, ...]) -> tuple[tuple[float, float, float], ...]:
    re_pattern = re.compile(r"\bRe\s*=\s*([0-9.eE+-]+)")
    limits = []
    for path in paths:
        reynolds = None
        alpha_values = []
        for line in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
            re_match = re_pattern.search(line)
            if re_match:
                try:
                    reynolds = float(re_match.group(1)) * 1e6
                except ValueError:
                    pass
            fields = line.split()
            if len(fields) < 3:
                continue
            try:
                alpha = float(fields[0])
                float(fields[1])
                float(fields[2])
                alpha_values.append(alpha)
            except ValueError:
                continue
        if reynolds is not None and alpha_values:
            limits.append((reynolds, min(alpha_values), max(alpha_values)))
    return tuple(sorted(limits))


def _operating_samples(
    rotor,
    performance,
    rho_kg_m3: float,
    dynamic_viscosity_pa_s: float,
) -> tuple[tuple[float, float], ...]:
    """Compute local Reynolds number and alpha at each qprop output element."""
    sections = rotor.sections
    section_count = int(rotor.nsections)
    if section_count < 2 or rho_kg_m3 <= 0 or dynamic_viscosity_pa_s <= 0:
        return ()
    samples = []
    for element in range(int(performance.nelems)):
        radius = float(performance.r[element])
        chord = None
        twist = None
        for index in range(section_count - 1):
            left, right = sections[index], sections[index + 1]
            if float(left.r) <= radius <= float(right.r):
                width = float(right.r) - float(left.r)
                ratio = (radius - float(left.r)) / width if width else 0.0
                chord = float(left.c) + ratio * (float(right.c) - float(left.c))
                twist = float(left.beta) + ratio * (float(right.beta) - float(left.beta))
                break
        if chord is None or twist is None:
            return ()
        reynolds = rho_kg_m3 * float(performance.W[element]) * chord / dynamic_viscosity_pa_s
        alpha_degrees = math.degrees(twist - float(performance.phi[element]))
        samples.append((reynolds, alpha_degrees))
    return tuple(samples)
