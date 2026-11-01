# PowerShell 5.1 entry layer. All native arguments use Windows argv quoting.
Set-StrictMode -Version Latest

function Initialize-ProjectBootstrap([string]$ProjectRoot) {
    $script:BootstrapRoot = [IO.Path]::GetFullPath($ProjectRoot).TrimEnd('\', '/')
    if ($script:BootstrapRoot.Split([char[]]@('\', '/')) -icontains '.codex') { throw 'Excluded directory in project path.' }
    foreach ($required in @('CMakeLists.txt', 'tools/bootstrap/project.py', 'tools/bootstrap/dependencies.lock.json')) {
        if (-not (Test-Path -LiteralPath (Join-Path $script:BootstrapRoot $required) -PathType Leaf)) {
            throw "ProjectRoot must contain a complete project checkout; missing $required."
        }
    }
    foreach ($directory in @('.local/tmp', '.local/cache', '.local/logs', '.local/state')) {
        New-Item -ItemType Directory -Force -Path (Join-Path $script:BootstrapRoot $directory) | Out-Null
    }
    New-Item -ItemType Directory -Force -Path 'D:/ENV/tmp' | Out-Null
    $env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
    $env:PIP_CACHE_DIR = Join-Path $script:BootstrapRoot '.local/cache/pip'
    $env:PYTHONDONTWRITEBYTECODE = '1'
    $env:PYTHONNOUSERSITE = '1'
}

function ConvertTo-WindowsArguments([string[]]$Values) {
    return (($Values | ForEach-Object {
        '"' + (($_ -replace '(\\*)"', '$1$1\"') -replace '(\\+)$', '$1$1') + '"'
    }) -join ' ')
}

function Invoke-BootstrapChild([string]$Program, [string[]]$Values, [switch]$Capture) {
    $child = New-Object Diagnostics.Process
    $child.StartInfo.FileName = $Program
    $child.StartInfo.Arguments = ConvertTo-WindowsArguments $Values
    $child.StartInfo.WorkingDirectory = $script:BootstrapRoot
    $child.StartInfo.UseShellExecute = $false
    $child.StartInfo.CreateNoWindow = $true
    $child.StartInfo.RedirectStandardOutput = $true
    $child.StartInfo.RedirectStandardError = $true
    $null = $child.Start()
    $errTask = $child.StandardError.ReadToEndAsync()
    if ($Capture) { $outTask = $child.StandardOutput.ReadToEndAsync() }
    else {
        while (-not $child.StandardOutput.EndOfStream) {
            [Console]::WriteLine($child.StandardOutput.ReadLine())
        }
    }
    $child.WaitForExit()
    $status = $child.ExitCode
    if ($Capture) {
        $result = @{ code = $status; output = $outTask.Result; error = $errTask.Result }
        $child.Dispose()
        return $result
    }
    if ($errTask.Result) { [Console]::Error.Write($errTask.Result) }
    $child.Dispose()
    return $status
}

function Test-ProjectPython([string]$Executable, [string]$Version) {
    if (-not $Executable -or -not (Test-Path -LiteralPath $Executable -PathType Leaf)) { return $false }
    # Do not invoke WindowsApps execution aliases, which can launch the Store.
    if ($Executable -match '\\WindowsApps\\') { return $false }
    $probe = 'import sys,sysconfig,pathlib; print(int(sys.implementation.name=="cpython" and ".".join(map(str,sys.version_info[:3]))=="' + $Version + '" and sys.maxsize>2**32 and pathlib.Path(sys.base_prefix,"python3.dll").is_file() and pathlib.Path(sysconfig.get_path("include"),"Python.h").is_file()))'
    try {
        $result = Invoke-BootstrapChild $Executable @('-B', '-c', $probe) -Capture
        return $result.code -eq 0 -and $result.output.Trim() -eq '1'
    } catch { return $false }
}

function Resolve-ProjectPython([string]$ExplicitPython, [bool]$Offline) {
    $lock = Get-Content -LiteralPath (Join-Path $script:BootstrapRoot 'tools/bootstrap/dependencies.lock.json') -Raw | ConvertFrom-Json
    $localPython = Join-Path $script:BootstrapRoot '.local/python/tools/python.exe'
    $candidates = @($ExplicitPython, $localPython, $env:FSIM_PYTHON_EXECUTABLE)
    foreach ($command in @('python.exe', 'python3.exe')) {
        $candidates += @(Get-Command $command -CommandType Application -ErrorAction SilentlyContinue | ForEach-Object { $_.Path })
    }
    # Read-only compatibility probe of the established installation; never install there.
    $candidates += 'D:/ENV/python/3.13/python.exe'
    foreach ($candidate in ($candidates | Where-Object { $_ } | Select-Object -Unique)) {
        if (Test-ProjectPython $candidate $lock.python.version) { return [IO.Path]::GetFullPath($candidate) }
    }
    if ($ExplicitPython) { throw "Explicit Python must be x64 CPython $($lock.python.version), with Python.h and python3.dll." }
    return Install-ProjectPython $Offline
}

function Install-ProjectPython([bool]$Offline) {
    $lock = Get-Content -LiteralPath (Join-Path $script:BootstrapRoot 'tools/bootstrap/dependencies.lock.json') -Raw | ConvertFrom-Json
    $localPython = Join-Path $script:BootstrapRoot '.local/python/tools/python.exe'
    $archive = Join-Path $script:BootstrapRoot ('.local/cache/' + $lock.python.filename)
    function Test-PythonArchive {
        if (-not (Test-Path -LiteralPath $archive -PathType Leaf)) { return $false }
        $stream = [IO.File]::OpenRead($archive)
        try { $hasher = [Security.Cryptography.SHA512]::Create(); $actual = [Convert]::ToBase64String($hasher.ComputeHash($stream)) }
        finally { $stream.Dispose(); if ($hasher) { $hasher.Dispose() } }
        return $actual -ceq $lock.python.sha512_base64
    }
    if (-not (Test-PythonArchive)) {
        if ($Offline) { throw "Pinned Python archive is missing or invalid: $archive" }
        [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
        $download = $archive + '.download'
        Invoke-WebRequest -UseBasicParsing -Uri $lock.python.url -OutFile $download
        Move-Item -LiteralPath $download -Destination $archive -Force
        if (-not (Test-PythonArchive)) { throw 'Pinned Python SHA512 mismatch.' }
    }
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $destination = Join-Path $script:BootstrapRoot '.local/python'
    New-Item -ItemType Directory -Force -Path $destination | Out-Null
    $zip = [IO.Compression.ZipFile]::OpenRead($archive)
    try {
        foreach ($entry in $zip.Entries) {
            $name = $entry.FullName.Replace('\', '/')
            if ($name.StartsWith('/') -or $name.Contains(':') -or $name.Split('/') -contains '..' -or $name.Split('/') -icontains '.codex') { throw "Unsafe Python archive entry: $name" }
            $target = [IO.Path]::GetFullPath((Join-Path $destination $name))
            if (-not $target.StartsWith($destination + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Archive entry leaves project.' }
            if ($name.EndsWith('/')) { New-Item -ItemType Directory -Force -Path $target | Out-Null; continue }
            New-Item -ItemType Directory -Force -Path ([IO.Path]::GetDirectoryName($target)) | Out-Null
            [IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $target, $true)
        }
    } finally { $zip.Dispose() }
    if (-not (Test-ProjectPython $localPython $lock.python.version)) { throw 'Project-local Python does not satisfy the pinned SDK contract.' }
    return $localPython
}

function Get-ManifestPython {
    $path = Join-Path $script:BootstrapRoot '.local/state/manifest.json'
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw 'Run setup.ps1 before build/run/stop.' }
    $manifest = Get-Content -LiteralPath $path -Raw | ConvertFrom-Json
    return $manifest.tools.python.path
}

function Invoke-ProjectBootstrap([string]$Python, [string[]]$Values) {
    return Invoke-BootstrapChild $Python (@('-B', (Join-Path $script:BootstrapRoot 'tools/bootstrap/project.py'), '--root', $script:BootstrapRoot) + $Values)
}
