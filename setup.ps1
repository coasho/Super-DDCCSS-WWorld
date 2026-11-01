[CmdletBinding()]
param(
    [string]$ProjectRoot,
    [string]$Python,
    [string]$Ucrt64Root,
    [switch]$Offline
)
$ErrorActionPreference = 'Stop'
if (-not $ProjectRoot) { $ProjectRoot = $PSScriptRoot }
try {
    . (Join-Path $PSScriptRoot 'tools/bootstrap/entry.ps1')
    Initialize-ProjectBootstrap $ProjectRoot
    $pythonPath = Resolve-ProjectPython $Python $Offline.IsPresent
    $arguments = @('setup')
    if ($Ucrt64Root) { $arguments += @('--ucrt64-root', $Ucrt64Root) }
    if ($Offline) { $arguments += '--offline' }
    exit (Invoke-ProjectBootstrap $pythonPath $arguments)
} catch { Write-Error $_; exit 1 }
