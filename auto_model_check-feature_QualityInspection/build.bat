@echo off
set VENV_PATH=venv
set PYTHON_PATH=%VENV_PATH%\Scripts\python.exe
set PYINSTALLER_PATH=%VENV_PATH%\Scripts\pyinstaller.exe
set OUTPUT_DIR=bin

for /f "delims=" %%a in ('%PYTHON_PATH% -c "import main; print(main.VERSION)"') do (
    set VERSION=%%a
)

set EXE_NAME=GeoEva_%VERSION%

mkdir %OUTPUT_DIR%

"%PYINSTALLER_PATH%" --onefile --console --onefile --hidden-import fiona --collect-submodules matplotlib --exclude-module pyinstaller --name %EXE_NAME% main.py --distpath %OUTPUT_DIR%

if %ERRORLEVEL% == 0 (
    echo EXE_NAME: %EXE_NAME%
    echo BuildSuccess!
    rd /s /q build
) else (
    echo BuildFailed!
)
