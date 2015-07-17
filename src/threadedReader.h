#ifndef THREADEDREADER_H
#define THREADEDREADER_H

#define BUFFER_LENGTH		50

#include <boost/asio.hpp>
//#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "Serial.h"
#include <queue>
#include <string>

class threadedReader
{
public:
	void start();
	std::string getNextLine();
	bool empty();
	threadedReader(std::string device, int baud);
	void stop();
	std::string getDevice();
	int getBaud();
private:
	static void readSerial(threadedReader *t);
	void push(std::string data);
	std::string lastString;
	std::queue<std::string> _queue;
    	mutable boost::mutex _mutex;
	boost::thread _readerThread;
	std::string _device;
	int _baud;
};

#endif

