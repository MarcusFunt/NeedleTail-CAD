param(
    [ValidateSet("pr3_baseline", "airfoil_candidate")]
    [string]$Geometry = "pr3_baseline",
    [string]$CaseDir
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$defaultCaseName = if ($Geometry -eq "pr3_baseline") { "su2-15mps" } else { "su2-15mps-$Geometry" }
$caseDir = if ($CaseDir) { [IO.Path]::GetFullPath($CaseDir) } else { Join-Path $repoRoot "tmp\$defaultCaseName" }
$python = Join-Path $repoRoot ".venv\Scripts\python.exe"
$configSource = Join-Path $PSScriptRoot "airframe_incompressible_euler.cfg"
$meshOutput = Join-Path $caseDir "airframe_domain.su2"

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

if (Test-Path -LiteralPath $caseDir) {
    $existingEntries = @(Get-ChildItem -LiteralPath $caseDir -Force)
    if ($existingEntries.Count -gt 0) {
        throw "Refusing to overwrite existing SU2 case outputs in $caseDir; select a fresh -CaseDir"
    }
}
New-Item -ItemType Directory -Path $caseDir -Force | Out-Null
& $python (Join-Path $PSScriptRoot "generate_mesh.py") --geometry $Geometry --output $meshOutput
if ($LASTEXITCODE -ne 0) {
    throw "Gmsh mesh generation failed with exit code $LASTEXITCODE"
}
$caseConfig = Join-Path $caseDir "case.cfg"
if (Test-Path -LiteralPath $caseConfig) {
    throw "Refusing to overwrite existing SU2 config: $caseConfig"
}
Copy-Item -LiteralPath $configSource -Destination $caseConfig

$env:SU2_RUN = $su2Run
$env:Path = "$su2Run;$env:Path"
Push-Location $caseDir
try {
    & $su2Executable "case.cfg"
    if ($LASTEXITCODE -ne 0) {
        throw "SU2_CFD failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
