param(
    [ValidateSet("pr3_baseline", "airfoil_candidate")]
    [string]$Geometry = "pr3_baseline",
    [double]$Rpm = 40000,
    [double]$Revolutions = 1.0,
    [int]$StepsPerRevolution = 120,
    [switch]$SkipMesh,
    [string]$CaseDir
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$defaultCaseName = if ($Geometry -eq "pr3_baseline") { "su2-rotor" } else { "su2-rotor-$Geometry" }
$caseDir = if ($CaseDir) { [IO.Path]::GetFullPath($CaseDir) } else { Join-Path $repoRoot "tmp\$defaultCaseName" }
$python = Join-Path $repoRoot ".venv\Scripts\python.exe"
$meshOutput = Join-Path $caseDir "rotor_domain.su2"

if (-not (Test-Path -LiteralPath $python)) {
    throw "Project Python environment missing. Run: uv sync --locked --extra aero --extra cfd-meshing"
}

$su2Run = [Environment]::GetEnvironmentVariable("SU2_RUN", "User")
if (-not $su2Run) {
    $su2Run = Join-Path $env:LOCALAPPDATA "Programs\SU2\8.5.0\bin"
}
$su2Executable = Join-Path $su2Run "SU2_CFD.exe"
if (-not (Test-Path -LiteralPath $su2Executable)) {
    throw "SU2_CFD.exe not found at $su2Executable"
}

$previousResults = @("rotor_case.csv", "solver.log", "rotor_history.csv", "config_check.log") |
    Where-Object { Test-Path -LiteralPath (Join-Path $caseDir $_) }
if ($previousResults.Count -gt 0) {
    throw "Refusing to overwrite prior SU2 outputs in ${caseDir}: $($previousResults -join ', ')"
}
if ($SkipMesh -and -not (Test-Path -LiteralPath $meshOutput)) {
    throw "-SkipMesh requires an existing mesh at $meshOutput"
}
New-Item -ItemType Directory -Path $caseDir -Force | Out-Null
if (-not $SkipMesh) {
    & $python (Join-Path $PSScriptRoot "generate_rotor_mesh.py") --geometry $Geometry --output $meshOutput
    if ($LASTEXITCODE -ne 0) {
        throw "Four-rotor sliding mesh generation failed with exit code $LASTEXITCODE"
    }
}

& $python (Join-Path $PSScriptRoot "configure_rotor_case.py") `
    --case-dir $caseDir --geometry $Geometry --rpm $Rpm --revolutions $Revolutions `
    --steps-per-revolution $StepsPerRevolution
if ($LASTEXITCODE -ne 0) {
    throw "SU2 rotating-case configuration failed with exit code $LASTEXITCODE"
}

$env:SU2_RUN = $su2Run
$env:Path = "$su2Run;$env:Path"
$env:OMP_NUM_THREADS = "4"
Push-Location $caseDir
try {
    & $su2Executable "rotor_case.cfg"
    if ($LASTEXITCODE -ne 0) {
        throw "SU2 rotating-propeller flow solve failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
