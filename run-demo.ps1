# Compiles and runs the IMU visualizer without Maven, using the dependency jars
# already in the local ~/.m2 repository. Defaults to the synthetic --demo feed so
# it runs with no hardware connected.
#
#   .\run-demo.ps1                  # mock feed
#   .\run-demo.ps1 --port COM9      # live serial feed
#   .\run-demo.ps1 --demo --slerp 1 # mock feed, no smoothing
#
# Pass any Main args after the script name; if none imply a source, --demo is used.

$ErrorActionPreference = 'Stop'
$proj = $PSScriptRoot
$m2 = "$env:USERPROFILE\.m2\repository"
$v = '3.3.6'

$jars = @(
  "$m2\org\lwjgl\lwjgl\$v\lwjgl-$v.jar",
  "$m2\org\lwjgl\lwjgl-glfw\$v\lwjgl-glfw-$v.jar",
  "$m2\org\lwjgl\lwjgl-opengl\$v\lwjgl-opengl-$v.jar",
  "$m2\org\lwjgl\lwjgl-stb\$v\lwjgl-stb-$v.jar",
  "$m2\org\lwjgl\lwjgl\$v\lwjgl-$v-natives-windows.jar",
  "$m2\org\lwjgl\lwjgl-glfw\$v\lwjgl-glfw-$v-natives-windows.jar",
  "$m2\org\lwjgl\lwjgl-opengl\$v\lwjgl-opengl-$v-natives-windows.jar",
  "$m2\org\lwjgl\lwjgl-stb\$v\lwjgl-stb-$v-natives-windows.jar",
  "$m2\com\fazecast\jSerialComm\2.11.2\jSerialComm-2.11.2.jar"
)
$cp = $jars -join ';'
$out = "$proj\target\classes"
New-Item -ItemType Directory -Force $out | Out-Null

$srcs = Get-ChildItem "$proj\src\main\java" -Recurse -Filter *.java | ForEach-Object { $_.FullName }
Write-Host "Compiling $($srcs.Count) sources..."
& javac --release 24 --enable-preview -cp $cp -d $out $srcs
if ($LASTEXITCODE -ne 0) { throw "compile failed" }

$runArgs = $args
if (-not $runArgs) { $runArgs = @('--demo') }
Write-Host "Running: Main $runArgs"
& java --enable-preview -cp "$out;$cp" com.kagenou.Avionics.Main @runArgs
