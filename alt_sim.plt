reset
set terminal qt font "Arial,16"

set grid
set xrange [0:20]
set multiplot layout 3,1
set ylabel '[V]'
set yrange[0:8]
unset xlabel
set ytics 2
plot "log/data00.log" u 1:2 w l lw 2 title "u"

unset xlabel
unset yrange
set ylabel '[m/s]'
set ytics 0.5
plot "log/data00.log" u 1:4 w l lw 2 title "w", "" u 1:6 w l lw 2 title "w\\_ref"

set xlabel 'Time[s]'
set ylabel '[m]'
set ytics 0.25
plot "log/data00.log" u 1:5 w l lw 2 title "z"

unset multiplot

pause -1