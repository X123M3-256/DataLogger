#include<cstddef>
#include<cstring>
#include<stdint.h>
#include <stdarg.h>

template <class T, int N> class ring_buffer
{
public:
	ring_buffer();
	size_t available();
	size_t free();
	size_t capacity();
	size_t discard(size_t len);
	void clear();
	size_t write(size_t (*source_func)(T*,size_t,void*),void* source_func_data,size_t len);
	size_t write(T* bytes,size_t len);
	size_t read(size_t (*dst_func)(T*,size_t,void*),void* dst_func_data,size_t len);
	size_t read(T* bytes,size_t len);
	size_t peek(T* bytes,size_t len);
	bool push(T item);
	bool push(T* item);
	bool pop(T* item);
private:
	T data[N];
	size_t head;
	size_t tail;
};

template <class T, int N> ring_buffer<T,N>::ring_buffer()
{
head=0;
tail=0;
}

template <class T, int N> size_t ring_buffer<T,N>::available()
{
	if(head<=tail)return (tail-head);
	else return N-(head-tail);
}

template <class T, int N> size_t ring_buffer<T,N>::free()
{
	if(head<=tail)return N-(tail-head)-1;
	else return (head-tail)-1;
}

template <class T, int N> size_t ring_buffer<T,N>::capacity() 
{
return N-1;
}

template <class T, int N> size_t ring_buffer<T,N>::discard(size_t len)
{
size_t avail=available();
	if(len==0||avail<len)len=avail;
head+=len;
	if(head>N)head-=N;
return len;
}

template <class T, int N> void ring_buffer<T,N>::clear()
{
head=0;
tail=0;
}

template <class T, int N> size_t ring_buffer<T,N>::write(size_t (*source_func)(T*,size_t,void*),void* source_func_data,size_t len)
{
	if(len==0)len=N;
size_t free_bytes=free();
	if(free_bytes<len)len=free_bytes;


size_t avail=available();
	if(avail==0)
	{
	head=0;
	tail=0;
	}
	
//Write data to buffer
size_t bytes_written=0;
	if(tail+len>N)
	{
	bytes_written=source_func(data+tail,N-tail,source_func_data);
		if(tail+bytes_written==N)bytes_written+=source_func(data,len-N+tail,source_func_data);
	}
	else bytes_written=source_func(data+tail,len,source_func_data);
//Update tail pointer
tail+=bytes_written;
	if(tail>=N)tail-=N;

return bytes_written;
}

template <class T, int N> size_t ring_buffer<T,N>::read(size_t (*dst_func)(T*,size_t,void*),void* dst_func_data,size_t len)
{
//Check available data in the buffer
size_t avail=available();
	if(avail==0)return 0;
	if(len==0||avail<len)len=avail;

//Read data from buffer
size_t bytes_read=0;
	if(head+len>N)
	{
	bytes_read=dst_func(data+head,N-head,dst_func_data);
		if(head+bytes_read>=N)bytes_read+=dst_func(data,len-N+head,dst_func_data);
	}
	else bytes_read+=dst_func(data+head,len,dst_func_data);

//Update head pointer
head+=bytes_read;
	if(head>=N)head-=N;

return bytes_read;
}

template <class T, int N> size_t ring_buffer<T,N>::peek(T* bytes,size_t len)
{
//Check available data in the buffer
size_t avail=available();
	if(avail==0)return 0;
	if(len==0||avail<len)len=avail;

	if(head+len>N)
	{
	memcpy(bytes,data+head,N-head);
	memcpy(bytes+N-head,data,len-N+head);
	}
	else
	{
	memcpy(bytes,data+head,len);
	}
return len;
}

template <class T, int N> bool ring_buffer<T,N>::push(T* item)
{
	if(free()==0)return false;
data[tail]=*item;
tail++;
	if(tail>=N)tail-=N;
return true;
}

template <class T, int N> bool ring_buffer<T,N>::pop(T* item)
{
	if(available()==0)return false;
*item=data[head];
head++;
	if(head>=N)head-=N;
return true;
}
