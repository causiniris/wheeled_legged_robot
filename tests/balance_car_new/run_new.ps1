param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$outputFolder = Join-Path $env:USERPROFILE '.pio\esp32-balance-new\host-tests'
New-Item -ItemType Directory -Force $outputFolder | Out-Null
$binary = Join-Path $outputFolder 'balance_behavior_new_test.exe'
& $Compiler '-std=c++17' '-Wall' '-Wextra' '-Werror' '-I' (Join-Path $PSScriptRoot 'fakes') (Join-Path $PSScriptRoot 'balance_behavior_new_test.cpp') '-o' $binary
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed.' }
& $binary
if ($LASTEXITCODE -ne 0) { throw 'Host test assertions failed.' }
