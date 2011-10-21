testClientBoost: testClientBoost.cc
	g++ -o testClientBoost testClientBoost.cc -L /usr/lib64/ -l boost_system -l boost_thread

testServerBoost: testServerBoost.cc simplepacket.o
	g++ -o testServerBoost testServerBoost.cc simplepacket.o -L /usr/lib64/ -l boost_system -l boost_thread

-cpp.:
	g++ -c $<

clean:
	rm -f testClientBoost
	rm -f testServerBoost
