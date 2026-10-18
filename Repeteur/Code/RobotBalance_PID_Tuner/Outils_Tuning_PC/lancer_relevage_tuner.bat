@echo off
chcp 65001 > nul
title RobotBalance — Banc d'Essais Relevage Automatique (Wi-Fi)

echo =======================================================
echo   RobotBalance — Relevage Automatique (Self-Recovery)
echo =======================================================
echo.
echo  AVANT DE LANCER :
echo    1. Allume le robot
echo    2. Sur ton PC, connecte-toi au WiFi :
echo         SSID         : RobotPID
echo         Mot de passe : robot1234
echo.
echo  Une fois connecte au WiFi RobotPID, appuie sur une touche.
echo =======================================================
pause > nul

python --version >nul 2>&1
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
python relevage_tuner_wifi.py

if errorlevel 1 (
    echo.
    echo [ERREUR] Le programme s'est termine avec une erreur.
    pause
)
