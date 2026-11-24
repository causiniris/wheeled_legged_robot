@echo off
cd /d "%~dp0"
python balance_curves.py
if errorlevel 1 pause
