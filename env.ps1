$VEX_PATH = "$($Env:USERPROFILE)\AppData\Roaming\Code\User\globalStorage\vexrobotics.vexcode"
$Env:PATH = $Env:PATH + ";$($VEX_PATH)\tools\cpp\toolchain_win32\clang\bin;$($VEX_PATH)\tools\cpp\toolchain_win32\tools\bin"
$Env:INC = "-I$($VEX_PATH)\sdk\cpp\V5\V5_20220726_10_00_00\vexv5\gcc\include" + 
" -I$($VEX_PATH)\sdk\cpp\V5\V5_20220726_10_00_00\vexv5\clang\8.0.0\include" + 
" -I$($VEX_PATH)\sdk\cpp\V5\V5_20220726_10_00_00\vexv5\include" + 
" -I$($VEX_PATH)\sdk\cpp\V5\V5_20220726_10_00_00\vexv5\gcc\include\c++\4.9.3" +
" -I$($VEX_PATH)\sdk\cpp\V5\V5_20220726_10_00_00\vexv5\gcc\include\c++\4.9.3\arm-none-eabi"
$Env:VEX_SDK_PATH = "$($VEX_PATH)\sdk\cpp\V5\V5_20220726_10_00_00\"