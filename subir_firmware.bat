@echo off
echo Copiando firmware y subiendo a Git...

:: Copiar el archivo binario
echo Copiando build/estacion_meteo.bin a firmware/firmware.bin...
cp build/estacion_meteo.bin firmware/firmware.bin

:: Verificar que el archivo se copió correctamente
if exist firmware/firmware.bin (
    echo Archivo copiado correctamente.
) else (
    echo Error: No se pudo copiar el archivo.
    pause
    exit /b 1
)

:: Agregar el archivo específico a Git
echo Agregando firmware/firmware.bin al repositorio...
git add firmware/firmware.bin

:: Hacer commit con mensaje fijo
echo Haciendo commit...
git commit -m "Firmware Update 2.0"

:: Subir cambios
echo Subiendo cambios al repositorio remoto...
git push origin main

echo ¡Proceso completado!
echo Archivo firmware.bin subido correctamente.
pause