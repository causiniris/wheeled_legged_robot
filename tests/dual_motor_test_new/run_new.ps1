param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputFolder = Join-Path $env:USERPROFILE '.pio\esp32-dual-motor-new\host-tests'
New-Item -ItemType Directory -Force $outputFolder | Out-Null
$binary = Join-Path $outputFolder 'dual_motor_test_new_test.exe'
$fakeIncludes = Join-Path $PSScriptRoot '..\balance_car_new\fakes'

& $compilerPath '-std=c++17' '-Wall' '-Wextra' '-Werror' '-I' $fakeIncludes `
  (Join-Path $PSScriptRoot 'dual_motor_test_new_test.cpp') '-o' $binary
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }

foreach ($scenario in @(
  'healthy',
  'encoder-failure',
  'magnet-failure',
  'serial-stop',
  'runtime-encoder-failure',
  'overspeed'
)) {
  & $binary $scenario
  if ($LASTEXITCODE -ne 0) { throw "Host test failed: $scenario" }
}
