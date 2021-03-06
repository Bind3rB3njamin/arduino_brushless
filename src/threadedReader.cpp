#include "threadedReader.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

threadedReader::threadedReader(std::string device, int baud)
{
	_device = device;
	_baud = baud;
	lastString = "";
}

void threadedReader::push(std::string data)
{
	boost::mutex::scoped_lock lock(_mutex);
	_queue.push(data);
}

void threadedReader::readSerial(threadedReader *t)
{
	Serial serial(t->getDevice(),t->getBaud());
	while(true)	
	{		
		try
		{
			boost::this_thread::interruption_point();
		}	
		catch(boost::thread_interrupted&)
        {
			serial.close();
            break;
        }

		std::string line = t->lastString;
		line.append(serial.readLine());
				

		//Because there can be read errors
		std::vector<std::string> tokens;
		boost::algorithm::split(tokens, line, boost::algorithm::is_any_of("\n"));
		
		if(tokens[tokens.size()-1].size() > 0)
		{
			t->lastString = tokens[tokens.size() - 1];
		}
		else
		{
			t->lastString = "";
		}
 
		for(int i = 0; i < tokens.size() - 1; i++)
		{
				t->push(tokens[i]);
		}	
	}
}
		

void threadedReader::start()
{
	_readerThread = boost::thread(boost::bind(readSerial, this));
}

void threadedReader::stop()
{
	_readerThread.interrupt();
}


std::string threadedReader::getNextLine()
{
	boost::mutex::scoped_lock lock(_mutex);
	std::string str = _queue.front();
	_queue.pop();
	return str;
}


bool threadedReader::empty()
{
  boost::mutex::scoped_lock lock(_mutex);
  return _queue.empty();
}


std::string threadedReader::getDevice()
{
	return _device;
}

int threadedReader::getBaud()
{
	return _baud;
}
