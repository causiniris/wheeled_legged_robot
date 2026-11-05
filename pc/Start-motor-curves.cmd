@echo off
cd /d "%~dp0"
python motor_curves.py
if errorlevel 1 pause
