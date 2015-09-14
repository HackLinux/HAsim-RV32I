all: algo

goodSim: GoodAlgo.c cLibSimulator.c
	gcc -o algo cLibSimulator.c GoodAlgo.c

baseSim: BrainDeadAlgo.c cLibSimulator.c
	gcc -o algo cLibSimulator.c BrainDeadAlgo.c

goodReal: GoodAlgo.c cLibReal.c
	gcc -o algo cLibReal.c GoodAlgo.c

baseReal: BrainDeadAlgo.c cLibReal.c
	gcc -o algo cLibReal.c BrainDeadAlgo.c

.PHONY: clean

clean:
	rm -rf ./algo ./program.hex ./a.out
