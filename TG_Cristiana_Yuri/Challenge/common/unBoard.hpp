#ifndef _UNBOARD_HPP_
#define _UNBOARD_HPP_

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

using namespace boost::interprocess;

template < class T >
class unBoard
{
	T *data;
	managed_shared_memory managed_shm;
	named_mutex mtx;

public:
	unBoard()
    :managed_shm(open_or_create, "unBoard", 1048576 /*1MB*/),
	mtx(open_or_create, typeid(T).name())
	{
		data = managed_shm.find_or_construct<T>(typeid(T).name())();
        mtx.unlock();
	}
    ~unBoard()
    {
        mtx.remove(typeid(T).name());
        managed_shm.destroy<T>(typeid(T).name());
    }

	void save(T teste)
	{
		mtx.lock();
		*data = teste;
		mtx.unlock();
	}

	T load()
	{
		return *data;
	}

};
/*
* g++ -c -Wall teste.cpp -lpthread
* g++ -L /lib -lpthread teste.o -o teste -lrt -lpthread
*/
#endif
