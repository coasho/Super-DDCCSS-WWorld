[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$ProjectRoot,
    [ValidateSet('viewer', 'demo', 'headless', 'hangar', 'python')][string]$Mode = 'viewer',
    [ValidateSet('ucrt64-release', 'ucrt64-headless')][string]$Preset = 'ucrt64-release',
    [switch]$Background,
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments = @()
)
$ErrorActionPreference = 'Stop'
if (-not $ProjectRoot) { $ProjectRoot = $PSScriptRoot }
try {
    . (Join-Path $PSScriptRoot 'tools/bootstrap/entry.ps1')
    Initialize-ProjectBootstrap $ProjectRoot
    $pythonPath = Get-ManifestPython
    $forward = @('run', '--mode', $Mode, '--preset', $Preset)
    if ($Background) { $forward += '--background' }
    $forward += '--'
    $forward += $Arguments
    exit (Invoke-ProjectBootstrap $pythonPath $forward)
} catch { Write-Error $_; exit 1 }
