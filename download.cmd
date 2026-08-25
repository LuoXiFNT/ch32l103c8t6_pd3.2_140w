set HEXFILE=%1

set "HEXFILE=%HEXFILE:\=/%"

openocd.exe -f ./tools/wch-target.cfg -c init -c halt -c "program %HEXFILE% 0x00000000 verify" -c wlink_reset_resume -c exit

