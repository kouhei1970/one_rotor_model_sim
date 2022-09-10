reset

set grid
set xrange [0:20]
set xlabel 'Time[s]'
set multiplot layout 3,1
set ylabel '[V]'
plot "log/data00.log" u 1:2 w l title "u"

set ylabel '[m/s]'
plot "log/data00.log" u 1:4 w l title "w"

set ylabel '[m]'
plot "log/data00.log" u 1:5 w l title "z"

unset multiplot

pause -1