# Windows PowerShell 5.1 is available on supported Windows hosts. This helper
# centralizes preset selection without needing a built executable or Python.
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
# Windows PowerShell's script binder splits colon-bearing arguments such as
# -DNAME:PATH=value. Read the process argv before that binding occurs; this
# also keeps empty arguments and application-owned options intact.
$processArgs = [Environment]::GetCommandLineArgs()
$fileIndex = [Array]::IndexOf($processArgs, '-File')
$inputArgs = @(if ($fileIndex -ge 0) { $processArgs | Select-Object -Skip ($fileIndex + 2) } else { $args })
$testbenchEntry = $inputArgs.Count -gt 0 -and $inputArgs[0] -eq '--testbench-entry'
if ($testbenchEntry) { $inputArgs = @($inputArgs | Select-Object -Skip 1) }
$preset = if ($env:FSIM_PRESET) { $env:FSIM_PRESET } else { 'ucrt64-headless' }
$index = 0
if ($inputArgs.Count -gt 0 -and $inputArgs[0] -eq '--preset') {
    if ($inputArgs.Count -lt 2 -or $inputArgs[1].StartsWith('-')) {
        Write-Output 'Usage: fsim [--preset NAME] COMMAND [ARGS...]'
        exit 2
    }
    $preset = $inputArgs[1]
    $index = 2
}
if ($index -lt $inputArgs.Count -and $inputArgs[$index] -eq '--') { $index++ }
if ($testbenchEntry) {
    $command = 'testbench'
} elseif ($index -lt $inputArgs.Count) {
    $command = $inputArgs[$index].ToLowerInvariant()
    $index++
} else {
    $command = 'help'
}
$forwardArgs = @($inputArgs | Select-Object -Skip $index)

try {
    $presets = Get-Content -LiteralPath (Join-Path $root 'CMakePresets.json') -Raw | ConvertFrom-Json
    $available = @($presets.configurePresets | Where-Object { -not $_.hidden } | ForEach-Object { $_.name })
    if ($preset -cnotin $available) {
        Write-Output "Unknown preset '$preset'. Available: $($available -join ', ')."
        exit 2
    }
    $build = Join-Path $root "build\$preset"
    $bin = Join-Path $build 'bin'
    $examples = Join-Path $build 'examples'
    $pythonMetadata = Join-Path $build 'python\fsim-python.json'
    $env:FSIM_PRESET = $preset
    $env:FSIM_BIN = $bin
    $toolchain = if ($env:FSIM_UCRT64_ROOT) { $env:FSIM_UCRT64_ROOT } else { 'D:\ENV\DevLanguages\Cpp\msys2\ucrt64' }
    $env:PATH = (Join-Path $toolchain 'bin') + ';' + $env:PATH
    $env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:\ENV\tmp'
    Set-Location -LiteralPath $root
    if ($command -in @('configure', 'build', 'dist')) {
        New-Item -ItemType Directory -Path $env:TEMP -Force | Out-Null
    }

    function Run-Child([string]$Program, [string[]]$Arguments) {
        Write-Host ('> "' + $Program + '" ' + (($Arguments | ForEach-Object { '"' + $_ + '"' }) -join ' '))
        # PS 5.1's native invocation drops empty strings and literal quotes.
        # Build the Windows argv encoding explicitly rather than rebinding it.
        $quotedArgs = @($Arguments | ForEach-Object {
            '"' + (($_ -replace '(\\*)"', '$1$1\"') -replace '(\\+)$', '$1$1') + '"'
        })
        $resolvedProgram = (Get-Command $Program -CommandType Application -ErrorAction Stop | Select-Object -First 1).Path
        $child = New-Object System.Diagnostics.Process
        $child.StartInfo.UseShellExecute = $false
        $child.StartInfo.WorkingDirectory = $root
        $child.StartInfo.FileName = $resolvedProgram
        $child.StartInfo.Arguments = $quotedArgs -join ' '
        $null = $child.Start()
        $child.WaitForExit()
        $status = $child.ExitCode
        $child.Dispose()
        return $status
    }

    function Configure([string[]]$Arguments) {
        $configureArgs = @('--preset', $preset)
        # The toolchain uses a cache variable. Honor the environment while
        # allowing an explicit -D setting to override it.
        if ($env:FSIM_UCRT64_ROOT -and -not ($Arguments -match '^-DFSIM_UCRT64_ROOT(?::[^=]+)?=')) {
            $configureArgs += '-DFSIM_UCRT64_ROOT=' + $env:FSIM_UCRT64_ROOT
        }
        return Run-Child 'cmake' ($configureArgs + $Arguments)
    }

    function Ensure-Configured {
        if (Test-Path -LiteralPath (Join-Path $build 'CMakeCache.txt')) { return 0 }
        return Configure @()
    }

    function Require-File([string]$Path) {
        if (Test-Path -LiteralPath $Path -PathType Leaf) { return $true }
        Write-Host "Preset '$preset' does not contain $Path"
        Write-Host "Build it with: fsim --preset $preset build"
        return $false
    }

    if ($command -in @('help', '--help', '-h')) {
        Write-Output @"
flightsim - a flight simulator for reinforcement learning

Usage: fsim [--preset NAME] COMMAND [ARGS...]
Preset: explicit --preset, then FSIM_PRESET, then ucrt64-headless.
Selected preset: $preset

  configure [CMAKE_ARGS]  configure the selected preset (accepts -D settings)
  build [BUILD_ARGS]      configure if needed, then build the selected preset
  test [CTEST_ARGS]       run the selected test preset
  dist [BUILD_ARGS]       configure if needed, then build its dist target
  where                  print the selected build directories, even unbuilt

  headless [ARGS]         run the command-line simulator
  train / fly / control  run the example trainers/controllers
  scenario <file>        run a scenario file
  python <script> [ARGS] run the selected Python SDK
  hangar [ARGS]          run the aircraft design tool
  testbench [ARGS]       run scenarios (--headless PREFIX, --list, --no-viewer)

  demo / viewer / replay / cameras / tiles
                         require a preset built with rendering enabled;
                         use --preset ucrt64-release or ucrt64-debug

Arguments after COMMAND are forwarded unchanged, including --preset and --.
Testbench.cmd accepts the same leading --preset option; no arguments opens
its GUI with a minimized console using the same preset selection.
"@
        exit 0
    }
    if ($command -eq 'where') {
        Write-Output "preset        $preset"
        Write-Output "applications  $bin"
        Write-Output "examples      $examples"
        Write-Output "tests         $(Join-Path $build 'tests')"
        Write-Output "python        $(Join-Path $build 'python')"
        Write-Output "packages      $(Join-Path $build 'dist')"
        exit 0
    }
    if ($command -eq 'configure') { exit (Configure $forwardArgs) }
    if ($command -in @('build', 'dist')) {
        $status = Ensure-Configured
        if ($status -ne 0) { exit $status }
        $buildArgs = @('--build', '--preset', $preset)
        if ($command -eq 'dist') { $buildArgs += @('--target', 'dist') }
        exit (Run-Child 'cmake' ($buildArgs + $forwardArgs))
    }
    if ($command -eq 'test') {
        if (-not (Test-Path -LiteralPath (Join-Path $build 'CMakeCache.txt'))) {
            Write-Output "Preset '$preset' is not configured. Run: fsim --preset $preset build"
            exit 1
        }
        exit (Run-Child 'ctest' (@('--preset', $preset) + $forwardArgs))
    }

    $applications = @{
        demo = @('bin\flightsim-viewer.exe', '--demo', '--vehicles', '6')
        viewer = @('bin\flightsim-viewer.exe')
        replay = @('bin\flightsim-viewer.exe', '--replay')
        train = @('examples\ppo_trainer.exe')
        fly = @('examples\minimal_trainer.exe')
        control = @('examples\multi_level_control.exe')
        scenario = @('examples\scenario_runner.exe')
        cameras = @('examples\vision_capture.exe', '--segmentation')
        headless = @('bin\flightsim.exe')
        tiles = @('bin\tile_prefetch.exe')
    }
    if ($applications.ContainsKey($command)) {
        $application = $applications[$command]
        $program = Join-Path $build $application[0]
        if (-not (Require-File $program)) { exit 1 }
        $applicationArgs = @($application | Select-Object -Skip 1)
        exit (Run-Child $program ($applicationArgs + $forwardArgs))
    }
    if ($command -in @('python', 'hangar', 'testbench')) {
        if (-not (Require-File $pythonMetadata)) {
            Write-Output 'The Python SDK needs CPython 3.11 or later (python.org or conda).'
            Write-Output "Configure it with: fsim --preset $preset configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe"
            exit 1
        }
        $python = (Get-Content -LiteralPath $pythonMetadata -Raw | ConvertFrom-Json).executable
        if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
            Write-Output "Configured Python interpreter not found: $python"
            exit 1
        }
        $env:PYTHONPATH = (Join-Path $build 'python') + ';' + $env:PYTHONPATH
        if ($command -eq 'hangar') {
            $env:PYTHONPATH = (Join-Path $root 'tools\hangar') + ';' + $env:PYTHONPATH
            $forwardArgs = @('-m', 'hangar') + $forwardArgs
        }
        if ($command -eq 'testbench') {
            $withoutViewer = @('--headless', '--no-viewer', '--list', '--help', '-h')
            $skipViewer = @($forwardArgs | Where-Object { $_ -in $withoutViewer -or $_.StartsWith('--headless=') }).Count -gt 0
            $viewerIndex = [Array]::IndexOf($forwardArgs, '--viewer')
            $viewer = Join-Path $bin 'flightsim-viewer.exe'
            if ($viewerIndex -ge 0 -and $viewerIndex + 1 -lt $forwardArgs.Count) { $viewer = $forwardArgs[$viewerIndex + 1] }
            $viewerOption = @($forwardArgs | Where-Object { $_.StartsWith('--viewer=') })
            if ($viewerOption.Count -gt 0) { $viewer = $viewerOption[-1].Substring(9) }
            $selectedPreset = $presets.configurePresets | Where-Object { $_.name -eq $preset }
            $headlessPreset = $selectedPreset.cacheVariables.FSIM_WITH_RENDER -eq 'OFF'
            $explicitViewer = $viewerIndex -ge 0 -or $viewerOption.Count -gt 0
            if (-not $skipViewer -and ($explicitViewer -or -not $headlessPreset) -and -not (Require-File $viewer)) { exit 1 }
            $env:PYTHONPATH = (Join-Path $root 'testbench') + ';' + $env:PYTHONPATH
            $forwardArgs = @('-m', 'fsim_testbench') + $forwardArgs
        }
        exit (Run-Child $python $forwardArgs)
    }
    Write-Output "Unknown command '$command'. Run fsim help."
    exit 2
} catch {
    Write-Output "Launcher failed: $($_.Exception.Message)"
    exit 1
}
