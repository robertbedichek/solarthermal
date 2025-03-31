
# # Date     Time Year Mode Tank Panel SpaT SpaH Spump Rpump Taka Call Open Clsd Time Erro
# Mar 24 20:05:21 2025 Oper  147   68   100    0     0     0    0    0    0    1    0    0

set key outside bottom
set key box

set terminal png size 1400, 300
set output 'solarthermal.png'
set xdata time
set timefmt "%b %d %H:%M:%S %Y"
set title "Solar-Thermal System" 
set ylabel "Fahrenheit" 
set yrange [30:180]
set xrange [*:*]
set xlabel " "
set grid
# set size 1.2 ,0.5
set key top left

heat_on(a, b, c, d) = int(a) | int(b) | int(c) | int(d)

plot 'solarthermal_log.txt' using 1:6 t "Tank" with lines lc rgb "green",  \
     'solarthermal_log.txt' using 1:7 t "Left Panel" with lines lc rgb "red", \
     'solarthermal_log.txt' using 1:8 t "Righ Panel" with lines lc rgb "orange", \
     'solarthermal_log.txt' using 1:9 title "Spa" with lines lc rgb "blue", \
     'solarthermal_log.txt' using 1:(($10 * 40)) title "Spa electric heat enabled" with points lc rgb "blue", \
     'solarthermal_log.txt' using 1:(($11 * 35)) title "Solar Pump" with points lc rgb "red", \
     'solarthermal_log.txt' using 1:(($12 * 45)) title "Recirc Pump" with points lc rgb "blue", \
     'solarthermal_log.txt' using 1:(($13 * 50)) title "Takagi" with points lc rgb "green", \
     'solarthermal_log.txt' using 1:(($15 * 55)) title "Valve Open" with points lc rgb "cyan"

