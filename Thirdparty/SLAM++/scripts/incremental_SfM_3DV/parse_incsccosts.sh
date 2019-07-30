#!/bin/bash

sed "s/[,)]//g" *.incsccosts.OU | egrep "(^L: )|(^l: )|(^E: )|(^B: )|(^rank of omega:)" | awk 'BEGIN { print "L flops,l flops,E flops,B flops,rank omega,rank lambda,originating verts"; nf = 0; } { if($1 == "L:") f = 0; else if($1 == "l:") f = 1; else if($1 == "E:") f = 2; else if($1 == "B:") f = 3; else if($1 == "rank") f = 4; else f = 666; if(f < 4) field[f] = $2; else if(f == 4) { field[4] = $4; field[5] = $9; field[6] = $12; } if(f != nf) nf = 0; else ++ nf; if(nf == 5) { printf("%s,%s,%s,%s,%s,%s,%s\n", field[0], field[1], field[2], field[3], field[4], field[5], field[6]); nf = 0; } }' > incsc_cost_anal.csv