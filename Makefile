
INC_BULLET=-I/usr/include/bullet
INC_OSG=-I/usr/include/osg
LD_OSG=-losg -losgViewer -losgSim -losgDB -losgGA -losgShadow

all: car carY car3

car: car.o btosg.o
	cc -O2 -o $@ $^ -L /home/jml/bullet3/src/BulletDynamics -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm
	

carY: carY.o btosg.o
	cc -O2 -o $@ $^ -L /home/jml/bullet3/src/BulletDynamics -l BulletDynamics -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm
	
car3: car.o btosg.o
	cc -O2 -o $@ $^ /home/jml/bullet3/src/BulletDynamics/libBulletDynamics.a -l BulletCollision -l LinearMath ${LD_OSG} -l stdc++ -lm

car.o: car.cpp btosg.h
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<

carY.o: carY.cpp btosg.h
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<

#-DBTOSG_SHADOW $<

btosg.o: btosg.cpp btosg.h
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} $<
	

CarHandlingDemo.o: CarHandlingDemo.cpp
	g++ -std=c++11 -Wall -O2 -c ${INC_BULLET} ${INC_OSG} -I /home/jml/bullet3/examples/CommonInterfaces -I /home/jml/bullet3/src $<



push: *.cpp *.h Makefile
	git add $^
	git commit -m "update"
	git push
