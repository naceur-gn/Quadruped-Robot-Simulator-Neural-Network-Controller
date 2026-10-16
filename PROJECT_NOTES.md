# PROJECT_NOTES.md

## Development environment (recorded 2026-10-16)

- OS: Microsoft Windows 11 Famille Unilingue (x64), version 10.0.26200
- Shell: Windows PowerShell 5.1
- Compiler: g++ 16.1.0 (MinGW-W64 x86_64-ucrt-posix-seh, Brecht Sanders r4), via WinGet `BrechtSanders.WinLibs.POSIX.UCRT`
  - Path: `C:\Users\gnich\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\g++.exe`
- CMake: 4.4.3
- Git: 2.55.0.windows.3
- Generator: `MinGW Makefiles` (NMake not available; CMake must be given
  `-G "MinGW Makefiles"` and `-DCMAKE_MAKE_PROGRAM=<...>\mingw32-make.exe`
  because CMake defaults to NMake on this machine).

## Notes / gotchas

- Smart App Control (SAC) is/enforced on this machine
  (`HKLM\SYSTEM\CurrentControlSet\Control\CI\Policy\VerifiedAndReputablePolicyState = 1`).
  It can block freshly compiled `.exe` files. Workaround observed: recompiling
  the file after a failed launch allowed it to run; `build\` outputs ran fine.
  If an executable is blocked with
  "Une stratégie de contrôle d'application a bloqué ce fichier",
  delete and rebuild it, or turn Smart App Control off in
  Windows Security → App & browser control.
- Line endings: Git warns LF→CRLF conversion on commit (normal on Windows).

## MuJoCo (to be filled in Task 1.1)

- Version: (not installed yet)
- Include dir: (TBD)
- Library dir: (TBD)
- Model XML + assets: (TBD)
