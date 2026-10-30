param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$outputFolder = Join-Path $env:USERPROFILE '.pio\esp32-balance-new\host-tests'
New-Item -ItemType Directory -Force $outputFolder | Out-Null
foreach ($test in @('software_i2c_new_test', 'balance_behavior_new_test')) {
    $binary = Join-Path $outputFolder ($test + '.exe')
    & $Compiler '-std=c++17' '-Wall' '-Wextra' '-Werror' '-I' (Join-Path $PSScriptRoot 'fakes') (Join-Path $PSScriptRoot ($test + '.cpp')) '-o' $binary
    if ($LASTEXITCODE -ne 0) { throw "Host test compilation failed: $test" }
    & $binary
    if ($LASTEXITCODE -ne 0) { throw "Host test assertions failed: $test" }
}
