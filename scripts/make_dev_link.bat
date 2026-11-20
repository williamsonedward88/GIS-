@echo off
REM 用法：修改下方 QGIS_PLUGINS 为你本机 QGIS 插件目录后运行
set QGIS_PLUGINS=%APPDATA%\QGIS\QGIS3\profiles\default\python\plugins
set PROJECT=%~dp0..\adaptive_lcp
if not exist "%QGIS_PLUGINS%" mkdir "%QGIS_PLUGINS%"
mklink /J "%QGIS_PLUGINS%\adaptive_lcp" "%PROJECT%"
echo 已建立软链接到 %QGIS_PLUGINS%
pause
