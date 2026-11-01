[CmdletBinding()]
param([string]$ProjectRoot)
$ErrorActionPreference = 'Stop'
if (-not $ProjectRoot) { $ProjectRoot = $PSScriptRoot }
try {
    . (Join-Path $PSScriptRoot 'tools/bootstrap/entry.ps1')
    Initialize-ProjectBootstrap $ProjectRoot
    exit (Invoke-ProjectBootstrap (Get-ManifestPython) @('stop'))
} catch { Write-Error $_; exit 1 }
