param([string]$SourceRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)))

# Exercise the real Windows batch entry points, without configuring the project
# or requiring its binaries. All compiler and process scratch stays on D:.
$ErrorActionPreference = 'Stop'
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:\ENV\tmp'
$fixture = Join-Path $env:TEMP ('fsim launcher contract ' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $fixture -Force | Out-Null
$fixture = (Resolve-Path -LiteralPath $fixture).Path
$log = Join-Path $fixture 'calls.jsonl'
$env:FSIM_STUB_LOG = $log
$env:FSIM_PRESET = ''
$env:FSIM_STUB_FAIL = ''
$env:FSIM_UCRT64_ROOT = Join-Path $fixture 'fake-toolchain'
$toolchainArg = '-DFSIM_UCRT64_ROOT=' + $env:FSIM_UCRT64_ROOT
$checks = 0

function Assert-Equal($Actual, $Expected, [string]$Message) {
    if (($Actual | ConvertTo-Json -Compress -Depth 5) -ne ($Expected | ConvertTo-Json -Compress -Depth 5)) {
        throw "$Message`nExpected: $($Expected | ConvertTo-Json -Compress)`nActual: $($Actual | ConvertTo-Json -Compress)"
    }
    $script:checks++
}

function Invoke-Launcher([string[]]$Arguments, [string]$Entry = 'fsim.cmd') {
    if (Test-Path -LiteralPath $log) { Remove-Item -LiteralPath $log }
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo.FileName = $env:ComSpec
    $quoted = @($Arguments | ForEach-Object {
        # Doubled quotes keep CMD metacharacters inside the quoted argument;
        # backslash-quote encoding alone toggles CMD's quote state.
        '"' + (($_ -replace '"', '""') -replace '(\\+)$', '$1$1') + '"'
    })
    $process.StartInfo.Arguments = '/d /s /c ""' + (Join-Path $fixture $Entry) + '" ' + ($quoted -join ' ') + '"'
    $process.StartInfo.WorkingDirectory = Join-Path $fixture 'outside'
    $process.StartInfo.UseShellExecute = $false
    $process.StartInfo.RedirectStandardOutput = $true
    $process.StartInfo.RedirectStandardError = $true
    $null = $process.Start()
    $stdout = $process.StandardOutput.ReadToEnd()
    $stderr = $process.StandardError.ReadToEnd()
    $process.WaitForExit()
    $calls = @()
    if (Test-Path -LiteralPath $log) { $calls = @(Get-Content -LiteralPath $log | ForEach-Object { $_ | ConvertFrom-Json }) }
    [pscustomobject]@{ Code = $process.ExitCode; Output = $stdout + $stderr; Calls = $calls }
}

function Stage-Preset([string]$Preset) {
    $stage = Join-Path $fixture "build\$Preset"
    foreach ($directory in 'bin', 'examples', 'python', 'tests') {
        New-Item -ItemType Directory -Path (Join-Path $stage $directory) -Force | Out-Null
    }
    foreach ($name in 'flightsim', 'flightsim-viewer', 'tile_prefetch') {
        Copy-Item -LiteralPath $stub -Destination (Join-Path $stage "bin\$name.exe")
    }
    foreach ($name in 'ppo_trainer', 'minimal_trainer', 'multi_level_control', 'scenario_runner', 'vision_capture') {
        Copy-Item -LiteralPath $stub -Destination (Join-Path $stage "examples\$name.exe")
    }
    Copy-Item -LiteralPath $stub -Destination (Join-Path $stage 'python\python-stub.exe')
    # The repository launcher must use native interpreter metadata instead of
    # reparsing arguments through this legacy direct-user batch entry point.
    Set-Content -LiteralPath (Join-Path $stage 'python\fsim-python.cmd') -Encoding ASCII -Value '@echo off', 'exit /b 93'
    @{ executable = (Join-Path $stage 'python\python-stub.exe') } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $stage 'python\fsim-python.json') -Encoding UTF8
}

try {
    # Use a real CMake configure to exercise stale generated Python entry
    # cleanup when the package is disabled or its interpreter probe fails.
    # These configurations have no native targets and never touch the repo's build.
    $realCMake = (Get-Command cmake -CommandType Application | Select-Object -First 1).Path
    $stateSource = Join-Path $fixture 'python entry state'
    New-Item -ItemType Directory -Path $stateSource | Out-Null
    $stateProject = @'
cmake_minimum_required(VERSION 3.25)
project(python_entry_state LANGUAGES NONE)
include("@SOURCE@/cmake/ResetPythonEntries.cmake")
if(FSIM_BUILD_PYTHON)
    set(FSIM_PYTHON_EXECUTABLE "${CMAKE_BINARY_DIR}/missing-python.exe" CACHE FILEPATH "" FORCE)
    add_subdirectory("@SOURCE@/python" python)
endif()
'@.Replace('@SOURCE@', $SourceRoot.Replace('\', '/'))
    Set-Content -LiteralPath (Join-Path $stateSource 'CMakeLists.txt') -Value $stateProject -Encoding ASCII
    foreach ($enabled in 'OFF', 'ON') {
        $stateBuild = Join-Path $stateSource "build-$enabled"
        $stateStage = Join-Path $stateBuild 'python'
        New-Item -ItemType Directory -Path $stateStage -Force | Out-Null
        foreach ($entry in 'fsim-python.json', 'fsim-python.cmd') {
            Set-Content -LiteralPath (Join-Path $stateStage $entry) -Value 'obsolete generated entry' -Encoding ASCII
        }
        $configureOutput = & $realCMake -S $stateSource -B $stateBuild "-DFSIM_BUILD_PYTHON=$enabled" 2>&1
        Assert-Equal $LASTEXITCODE 0 "Python entry-state fixture configures successfully ($enabled): $configureOutput"
        foreach ($entry in 'fsim-python.json', 'fsim-python.cmd') {
            Assert-Equal (Test-Path -LiteralPath (Join-Path $stateStage $entry)) $false "Python $enabled must remove obsolete generated $entry before configure/probe."
        }
    }
    foreach ($file in 'fsim.cmd', 'Testbench.cmd', 'CMakePresets.json') {
        Copy-Item -LiteralPath (Join-Path $SourceRoot $file) -Destination $fixture
    }
    if (Test-Path -LiteralPath (Join-Path $SourceRoot 'tools\launch')) {
        New-Item -ItemType Directory -Path (Join-Path $fixture 'tools') -Force | Out-Null
        Copy-Item -LiteralPath (Join-Path $SourceRoot 'tools\launch') -Destination (Join-Path $fixture 'tools') -Recurse
    }
    New-Item -ItemType Directory -Path (Join-Path $fixture 'outside'), (Join-Path $fixture 'fake-toolchain\bin') -Force | Out-Null
    $stub = Join-Path $fixture 'stub.exe'
    Add-Type -OutputAssembly $stub -OutputType ConsoleApplication -TypeDefinition @'
using System;
using System.IO;
using System.Diagnostics;
using System.Linq;
class LauncherStub {
    static string Quote(string value) {
        return "\"" + (value ?? "").Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\r", "\\r").Replace("\n", "\\n") + "\"";
    }
    static int Main(string[] args) {
        string exe = Process.GetCurrentProcess().MainModule.FileName;
        string name = Path.GetFileNameWithoutExtension(exe);
        string action = name == "cmake" ? (args.Contains("--build") ? (args.Contains("dist") ? "dist" : "build") : "configure") : name;
        string line = "{\"name\":" + Quote(name) + ",\"exe\":" + Quote(exe) + ",\"cwd\":" + Quote(Environment.CurrentDirectory) +
            ",\"preset\":" + Quote(Environment.GetEnvironmentVariable("FSIM_PRESET")) + ",\"bin\":" + Quote(Environment.GetEnvironmentVariable("FSIM_BIN")) +
            ",\"args\":[" + string.Join(",", args.Select(Quote).ToArray()) + "]}";
        File.AppendAllText(Environment.GetEnvironmentVariable("FSIM_STUB_LOG"), line + "\n");
        if (Environment.GetEnvironmentVariable("FSIM_STUB_FAIL") == action) return 37;
        if (action == "configure") {
            int index = Array.IndexOf(args, "--preset");
            if (index >= 0 && index + 1 < args.Length) {
                string path = Path.Combine(Environment.CurrentDirectory, "build", args[index + 1]);
                Directory.CreateDirectory(path);
                File.WriteAllText(Path.Combine(path, "CMakeCache.txt"), "stub cache");
            }
        }
        return 0;
    }
}
'@
    foreach ($name in 'cmake', 'ctest') { Copy-Item -LiteralPath $stub -Destination (Join-Path $fixture "fake-toolchain\bin\$name.exe") }
    $env:PATH = (Join-Path $fixture 'fake-toolchain\bin') + ';' + $env:PATH

    # The previous launcher fails here: it requires an existing executable
    # before even dispatching build. There are intentionally no build binaries.
    $result = Invoke-Launcher @('build', '--parallel', '2')
    Assert-Equal $result.Code 0 "First build must work without any executable. $($result.Output)"
    Assert-Equal $result.Calls.Count 2 'First build must configure then build.'
    Assert-Equal @($result.Calls[0].args) @('--preset', 'ucrt64-headless', $toolchainArg) 'Configure uses the fixed default and environment toolchain root.'
    Assert-Equal @($result.Calls[1].args) @('--build', '--preset', 'ucrt64-headless', '--parallel', '2') 'Build uses the same preset and forwards options.'
    Assert-Equal $result.Calls[0].cwd $fixture 'Commands run from the repository even when invoked elsewhere.'

    $result = Invoke-Launcher @('help')
    Assert-Equal $result.Code 0 'Help works without applications.'
    Assert-Equal $result.Calls.Count 0 'Help does not execute a child.'
    $result = Invoke-Launcher @('where')
    Assert-Equal $result.Code 0 'Where works without applications.'
    if ($result.Output -notlike '*build\ucrt64-headless\bin*' -or $result.Output -like '*build\ucrt64-release*') { throw 'Where must show only the selected preset.' }
    $checks++
    $result = Invoke-Launcher @('--preset', 'ucrt64-debug', 'test')
    Assert-Equal $result.Code 1 'An unconfigured test preset fails with a build hint.'
    Assert-Equal $result.Calls.Count 0 'Unconfigured tests never call ctest.'
    if ($result.Output -notlike '*fsim --preset ucrt64-debug build*') { throw 'Unconfigured test error must tell the user how to build it.' }
    $checks++

    $env:FSIM_PRESET = 'ucrt64-debug'
    $result = Invoke-Launcher @('--preset', 'ucrt64-release', 'build')
    Assert-Equal @($result.Calls[0].args) @('--preset', 'ucrt64-release', $toolchainArg) 'Explicit preset overrides the environment.'
    Assert-Equal @($result.Calls[1].args) @('--build', '--preset', 'ucrt64-release') 'Build never mixes presets.'
    $result = Invoke-Launcher @('configure', '-DFSIM_PYTHON_EXECUTABLE=D:/some path/python.exe')
    Assert-Equal @($result.Calls[0].args) @('--preset', 'ucrt64-debug', $toolchainArg, '-DFSIM_PYTHON_EXECUTABLE=D:/some path/python.exe') 'Configure uses the environment and preserves a value containing spaces.'
    $result = Invoke-Launcher @('configure', '-DFSIM_UCRT64_ROOT:PATH=D:/explicit toolchain')
    Assert-Equal @($result.Calls[0].args) @('--preset', 'ucrt64-debug', '-DFSIM_UCRT64_ROOT:PATH=D:/explicit toolchain') 'Explicit toolchain cache setting overrides the environment.'
    $env:FSIM_PRESET = ''
    $result = Invoke-Launcher @('build')
    Assert-Equal $result.Calls.Count 1 'Configured builds do not configure again.'
    $result = Invoke-Launcher @('test', '-R', 'a test name')
    Assert-Equal @($result.Calls[0].args) @('--preset', 'ucrt64-headless', '-R', 'a test name') 'Test uses the selected preset and forwards arguments.'
    $result = Invoke-Launcher @('dist')
    Assert-Equal @($result.Calls[0].args) @('--build', '--preset', 'ucrt64-headless', '--target', 'dist') 'Dist uses the selected build preset.'

    foreach ($arguments in @(@('--preset', 'missing', 'build'), @('--preset', 'UCRT64-HEADLESS', 'build'), @('--preset'), @('missing-command'))) {
        $result = Invoke-Launcher $arguments
        Assert-Equal $result.Code 2 'Invalid launcher input returns usage failure.'
        Assert-Equal $result.Calls.Count 0 'Invalid input never executes a child.'
    }
    foreach ($action in 'configure', 'build', 'ctest', 'dist') {
        $env:FSIM_STUB_FAIL = $action
        $command = if ($action -eq 'ctest') { 'test' } else { $action }
        $result = Invoke-Launcher @($command)
        Assert-Equal $result.Code 37 "$command preserves its child exit code."
    }
    $env:FSIM_STUB_FAIL = 'configure'
    Remove-Item -LiteralPath (Join-Path $fixture 'build\ucrt64-debug\CMakeCache.txt')
    $result = Invoke-Launcher @('--preset', 'ucrt64-debug', 'build')
    Assert-Equal $result.Code 37 'Auto-configure failure propagates.'
    Assert-Equal $result.Calls.Count 1 'Failed configure never starts build.'
    $env:FSIM_STUB_FAIL = ''
    $result = Invoke-Launcher @('--preset', 'ucrt64-debug', 'dist')
    Assert-Equal $result.Calls.Count 2 'First dist automatically configures.'
    Assert-Equal @($result.Calls[1].args) @('--build', '--preset', 'ucrt64-debug', '--target', 'dist') 'First dist stays on the selected preset.'

    foreach ($preset in 'ucrt64-headless', 'ucrt64-release', 'ucrt64-debug') { Stage-Preset $preset }
    Remove-Item -LiteralPath (Join-Path $fixture 'build\ucrt64-headless\bin\flightsim-viewer.exe')
    foreach ($command in 'viewer', 'demo', 'replay') {
        $result = Invoke-Launcher @($command)
        Assert-Equal $result.Code 1 'Missing viewer is a clear failure.'
        Assert-Equal $result.Calls.Count 0 'A missing headless viewer never falls back to release/debug.'
        if ($result.Output -notlike '*ucrt64-headless*' -or $result.Output -notlike '*flightsim-viewer.exe*') { throw 'Missing viewer message identifies the preset and executable.' }
        $checks++
    }
    $result = Invoke-Launcher @('headless', '--input', 'path with spaces!', '--', '--preset', 'application-owned')
    Assert-Equal @($result.Calls[0].args) @('--input', 'path with spaces!', '--', '--preset', 'application-owned') 'Arguments after the command belong to the application.'
    Assert-Equal $result.Calls[0].exe (Join-Path $fixture 'build\ucrt64-headless\bin\flightsim.exe') 'Application routing uses exactly the selected preset.'
    $exactArgs = @('', 'literal"quote', 'two "quoted words"', 'space and ampersand & bang!', 'two "a&b|c>d^e" words', '50% complete', 'C:\path with spaces\')
    $result = Invoke-Launcher (@('headless') + $exactArgs)
    Assert-Equal @($result.Calls[0].args) $exactArgs 'Native forwarding preserves empty strings, literal quotes and trailing backslashes.'
    $result = Invoke-Launcher @('--preset', 'ucrt64-release', 'scenario', 'scenario file.json')
    Assert-Equal $result.Calls[0].exe (Join-Path $fixture 'build\ucrt64-release\examples\scenario_runner.exe') 'Example routing uses exactly the selected preset.'
    Assert-Equal @($result.Calls[0].args) @('scenario file.json') 'Example paths with spaces arrive intact.'
    $env:FSIM_STUB_FAIL = 'flightsim'
    $result = Invoke-Launcher @('headless')
    Assert-Equal $result.Code 37 'Application failures propagate.'
    $env:FSIM_STUB_FAIL = ''

    $result = Invoke-Launcher @('python', 'script file.py', '--preset', 'application-owned')
    Assert-Equal @($result.Calls[0].args) @('script file.py', '--preset', 'application-owned') 'Python forwards its arguments and uses the selected stage.'
    Assert-Equal $result.Calls[0].exe (Join-Path $fixture 'build\ucrt64-headless\python\python-stub.exe') 'Python does not select another SDK.'
    $result = Invoke-Launcher (@('python') + $exactArgs)
    Assert-Equal @($result.Calls[0].args) $exactArgs 'Staged Python forwarding preserves empty values and combined quotes/metacharacters.'
    $result = Invoke-Launcher @('testbench')
    Assert-Equal $result.Code 0 'Headless presets can open the testbench UI without a native viewer.'
    Assert-Equal $result.Calls[0].bin (Join-Path $fixture 'build\ucrt64-headless\bin') 'Headless GUI stays on the selected build.'
    $result = Invoke-Launcher @('testbench', '--viewer', 'missing explicit viewer.exe')
    Assert-Equal $result.Code 1 'An explicit missing testbench viewer fails clearly.'
    Assert-Equal $result.Calls.Count 0 'An explicit viewer never falls back to another file.'
    Remove-Item -LiteralPath (Join-Path $fixture 'build\ucrt64-debug\bin\flightsim-viewer.exe')
    $result = Invoke-Launcher @('--preset', 'ucrt64-debug', 'testbench')
    Assert-Equal $result.Code 1 'Rendering presets report an unexpectedly missing viewer.'
    Assert-Equal $result.Calls.Count 0 'Rendering presets never fall back to another viewer.'
    foreach ($mode in @(@('--headless', 'fa9.'), @('--no-viewer'), @('--list'), @('--help'))) {
        $result = Invoke-Launcher (@('testbench') + $mode)
        Assert-Equal $result.Code 0 'Explicit non-viewer testbench modes work in a headless preset.'
        Assert-Equal $result.Calls[0].bin (Join-Path $fixture 'build\ucrt64-headless\bin') 'Testbench receives the selected FSIM_BIN.'
        Assert-Equal @($result.Calls[0].args) (@('-m', 'fsim_testbench') + $mode) 'Testbench receives the original options.'
    }
    $env:FSIM_STUB_FAIL = 'python-stub'
    $result = Invoke-Launcher @('--preset', 'ucrt64-headless', '--headless', 'fa9.', '--option', 'label=two words') 'Testbench.cmd'
    Assert-Equal $result.Code 37 'Explicit Testbench invocation runs synchronously and preserves its exit status.'
    Assert-Equal @($result.Calls[0].args) @('-m', 'fsim_testbench', '--headless', 'fa9.', '--option', 'label=two words') 'Testbench wrapper preserves spaces and options.'
    $env:FSIM_STUB_FAIL = ''

    Write-Output "Launcher contract: $checks checks passed."
} finally {
    # Resolve before deletion: this fixture must stay beneath the named D: scratch directory.
    $resolved = [IO.Path]::GetFullPath($fixture)
    if ($resolved.StartsWith('D:\ENV\tmp\fsim launcher contract ', [StringComparison]::OrdinalIgnoreCase)) {
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
