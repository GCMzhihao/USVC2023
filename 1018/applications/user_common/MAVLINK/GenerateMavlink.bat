rd /s /q C
rd /s /q CS
xcopy pymavlink\CS CS /s /i /y
for %%i in (*.xml) do (  
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=1.0 --output=.\C %%i
python -m pymavlink.tools.mavgen --lang=CS --wire-protocol=1.0 --output=.\CS\ %%i)
pause