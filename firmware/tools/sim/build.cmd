@echo off
rem Builds the host tools with MSVC (Visual Studio Build Tools 2022, C++ workload):
rem   out\test_model.exe  firmware model + controller unit test (no MuJoCo needed)
rem   out\raft_sim.exe    firmware controller driving the MuJoCo arm through a simulated servo bus
rem   out\test_rti.exe, out\test_app.exe          PAR6 link and application layer tests (no MuJoCo)
rem   out\bilateral_sim.exe, out\calib_sim.exe    teleoperation and bench calibration rehearsals (MuJoCo)
rem   out\rti_probe.exe, out\fake_rcb_server.exe  PAR6 link over real sockets (docs/rcb_link_test.md)
rem   out\test_web.exe    web UI server test (no MuJoCo)
rem   out\web_sim.exe     the web UI on this PC against the MuJoCo arm: http://localhost:8080/ (docs/web_ui.md)
rem MuJoCo: set MUJOCO_DIR to a folder holding include\mujoco\mujoco.h and mujoco.dll, or have the
rem MuJoCo Python package installed (pip install mujoco); its folder is used then.
setlocal
set "HERE=%~dp0"
set "SRC=%HERE%..\..\src\main"
set "OUT=%HERE%out"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR (echo Visual Studio C++ build tools not found. & exit /b 1)
set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
call "%VSDIR%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1

if not exist "%OUT%" mkdir "%OUT%"
set "FW=%SRC%\kinematics.cpp %SRC%\dynamics.cpp %SRC%\control.cpp %SRC%\model_selftest.cpp %SRC%\rti_protocol.cpp %SRC%\rti_link.cpp %SRC%\teleop.cpp %SRC%\config_store.cpp %SRC%\calibration.cpp %SRC%\leader_frame.cpp %SRC%\json_lite.cpp %SRC%\app.cpp %SRC%\web_server.cpp %SRC%\web_bridge.cpp %SRC%\servo_info.cpp %SRC%\boot.cpp %SRC%\loop_stats.cpp"
set "FLAGS=/nologo /std:c++17 /O2 /EHsc /W4 /wd4244 /wd4305 /D_CRT_SECURE_NO_WARNINGS /I"%SRC%""

cl %FLAGS% "%HERE%test_model.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\test_model.exe" || exit /b 1
cl %FLAGS% "%HERE%test_rti.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\test_rti.exe" || exit /b 1
cl %FLAGS% "%HERE%test_app.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\test_app.exe" || exit /b 1
cl %FLAGS% "%HERE%test_web.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\test_web.exe" || exit /b 1
cl %FLAGS% "%HERE%rti_probe.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\rti_probe.exe" /link ws2_32.lib winmm.lib || exit /b 1
cl %FLAGS% "%HERE%fake_rcb_server.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\fake_rcb_server.exe" /link ws2_32.lib winmm.lib || exit /b 1

if not defined MUJOCO_DIR (
  for /f "usebackq tokens=*" %%i in (`python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))" 2^>nul`) do set "MUJOCO_DIR=%%i"
)
if not defined MUJOCO_DIR (echo MuJoCo not found: set MUJOCO_DIR or pip install mujoco. test_model.exe was built. & exit /b 1)
lib /nologo /def:"%HERE%mujoco_min.def" /out:"%OUT%\mujoco.lib" /machine:x64 >nul || exit /b 1
copy /y "%MUJOCO_DIR%\mujoco.dll" "%OUT%\" >nul || exit /b 1
cl %FLAGS% /I"%MUJOCO_DIR%\include" "%HERE%raft_sim.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\raft_sim.exe" /link "%OUT%\mujoco.lib" || exit /b 1
cl %FLAGS% /I"%MUJOCO_DIR%\include" "%HERE%bilateral_sim.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\bilateral_sim.exe" /link "%OUT%\mujoco.lib" || exit /b 1
cl %FLAGS% /I"%MUJOCO_DIR%\include" "%HERE%calib_sim.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\calib_sim.exe" /link "%OUT%\mujoco.lib" || exit /b 1
python "%HERE%..\build_web.py" || exit /b 1
cl %FLAGS% /I"%MUJOCO_DIR%\include" "%HERE%web_sim.cpp" %FW% /Fo"%OUT%/" /Fe"%OUT%\web_sim.exe" /link "%OUT%\mujoco.lib" ws2_32.lib winmm.lib || exit /b 1
echo Built test_model, test_rti, test_app, test_web, rti_probe, fake_rcb_server, raft_sim, bilateral_sim, calib_sim and web_sim in %OUT%
