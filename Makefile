
INC_BULLET=-I/usr/include/bullet
INC_OSG=-I/usr/include/osg
LD_OSG=-losg -losgViewer -losgSim -losgDB -losgGA -losgShadow

all: carZ carY 

carZ: carZ.o btosg.o
	cc -O2 -o $@ $^ -L /home/jml/bullet3/src/BulletDynamics -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm
	

carY: carY.o btosg.o
	cc -O2 -o $@ $^ -L /home/jml/bullet3/src/BulletDynamics -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm
	
carZ.o: carZ.cpp btosg.h btosgVehicle.h
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<

carY.o: carY.cpp btosg.h btosgVehicle.h
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<
	

push: *.cpp *.h Makefile README.md
	git add $^
	git commit -m "update"
	git push
