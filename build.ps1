[CmdletBinding()]
param(
    [string]$ProjectRoot,
    [string[]]$Models = @('all'),
    [ValidateRange(1, 128)][int]$Jobs = [Math]::Max(1, [Math]::Min(16, [Environment]::ProcessorCount)),
    [ValidateRange(1, 2147483647)][int]$MemoryLimitMiB = 4096,
    [switch]$Test,
    [switch]$Plan
)
$ErrorActionPreference = 'Stop'
if (-not $ProjectRoot) { $ProjectRoot = $PSScriptRoot }
try {
    . (Join-Path $PSScriptRoot 'tools/bootstrap/entry.ps1')
    Initialize-ProjectBootstrap $ProjectRoot
    $pythonPath = Get-ManifestPython
    $arguments = @('build', '--jobs', [string]$Jobs,
                   '--memory-limit-mib', [string]$MemoryLimitMiB)
    foreach ($model in $Models) { $arguments += @('--model', $model) }
    if ($Test) { $arguments += '--test' }
    if ($Plan) { $arguments += '--plan' }
    exit (Invoke-ProjectBootstrap $pythonPath $arguments)
} catch { Write-Error $_; exit 1 }
