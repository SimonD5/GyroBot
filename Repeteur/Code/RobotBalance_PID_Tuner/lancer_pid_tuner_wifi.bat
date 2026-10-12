@echo off
chcp 65001 > nul
title RobotBalance PID Tuner WiFi

echo ============================================
echo   RobotBalance PID Tuner - Version WiFi
echo ============================================
echo.
echo  AVANT DE LANCER :
echo    1. Connecte le PC au reseau WiFi du robot
echo       SSID     : RobotPID
echo       Mot de passe : robot1234
echo.
echo    2. IP du robot : 192.168.4.1  Port : 3333
echo.
echo  Lancement en cours...
echo.

where python >nul 2>&1
if errorlevel 1 (
    echo [ERREUR] Python introuvable. Installe Python 3.10+
    echo         https://www.python.org/downloads/
    pause
    exit /b 1
)

python -c "import matplotlib" >nul 2>&1
if errorlevel 1 (
    echo [INFO] Installation de matplotlib...
    python -m pip install matplotlib --quiet
)

cd /d "%~dp0"
python pid_tuner_wifi.py

if errorlevel 1 (
    echo.
    echo [ERREUR] Le programme s'est termine avec une erreur.
    pause
)
