openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Release/Drone.elf verify reset exit"
