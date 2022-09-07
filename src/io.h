#include<stdint.h>
#include <stdarg.h>
#include "SdFat/SdFat.h"
#include "ring_buffer.h"

#define READ_BUFFER_SIZE 256
#define WRITE_BUFFER_SIZE 256

#define IO_READ_ENABLE 0x1
#define IO_WRITE_ENABLE 0x2
#define IO_SEEK_ENABLE 0x4
#define IO_WRITE_BUFFER 0x8
#define IO_READ_BUFFER 0x10
#define IO_OPEN 0x20

#define O_READ_ONLY IO_READ_ENABLE
#define O_WRITE_ONLY IO_WRITE_ENABLE
#define O_READ_WRITE (IO_READ_ENABLE|IO_WRITE_ENABLE)
#define O_CREATE 0x80

class io_stream
{
public:
	io_stream();

	bool is_writeable();
	bool is_readable();

	size_t available();
	size_t read(uint8_t* buf,size_t len);
	size_t peek(uint8_t* buf,size_t len);
	size_t write(uint8_t* buf,size_t len);
	size_t write(const char* buf,size_t len);
	size_t write(const char* buf);
	size_t write(const char c);

	//TODO would templates help here?
	uint8_t read_uint8();
	uint16_t read_uint16();
	uint32_t read_uint32();
	int8_t read_int8();
	int16_t read_int16();
	int32_t read_int32();
	float read_float();
	double read_double();

	size_t vwritef(const char* fmt,va_list args);
	size_t writef(const char* fmt,...);
	size_t discard(size_t len);
	void flush();
	

	//TODO see if I can make these private
	virtual size_t write_raw(const uint8_t*,size_t)=0;
	virtual size_t read_raw(uint8_t*,size_t)=0;
	virtual size_t available_raw()=0;
	virtual void flush_raw()=0;
protected:
	uint32_t flags;
private:
	ring_buffer<uint8_t,READ_BUFFER_SIZE> read_buffer;
	ring_buffer<uint8_t,WRITE_BUFFER_SIZE> write_buffer;

};

class io_serial:public io_stream
{
public:
	bool open(uint32_t baud,uint32_t open_flags);
	void close();
	virtual uint32_t id();
	virtual bool begin(uint32_t baud);
	virtual void end();
	virtual size_t write_raw(const uint8_t*,size_t);
	virtual size_t read_raw(uint8_t*,size_t);
	virtual size_t available_raw();
	virtual void flush_raw();
};



extern io_serial* usb_serial;
extern io_serial* serial_ports[2];


enum
{
ERROR_NONE=0,
ERROR_DOES_NOT_EXIST=1,
ERROR_IS_DIRECTORY=2,
ERROR_INVALID_MODE=3,
ERROR_PATH_NOT_SET=4,
ERROR_ALREADY_EXISTS=5,
ERROR_IS_FILE=6,
ERROR_UNSPECIFIED=7
};

const char* io_get_error_string(int err);

class io_file:public io_stream
{
public:
	io_file();
	io_file(const char* name);
	virtual int open(uint32_t open_flags);
	int open(const char* name,uint32_t open_flags);
	int remove();
	int remove(const char* name);
	virtual void close();
	virtual size_t write_raw(const uint8_t*,size_t);
	virtual size_t read_raw(uint8_t*,size_t);
	virtual size_t available_raw();
	virtual void flush_raw();
protected:
	const char* path;
	SdFile file;
};

class io_dir
{
public:
	io_dir();
	io_dir(const char* name);
	int open();
	int open(const char* name);
	int create();
	int create(const char* name);
	int remove();
	int remove(const char* name);
	void close();
	bool get_next_entry(char* name,int len,bool* is_directory);
private:
	uint16_t flags;
	const char* path;
	SdFile dir;
	int open(int sd_mode);
};

bool io_init();


enum
{
LOG_INFO=0x1,
LOG_ERROR=0x2,
LOG_DEBUG=0x4,
LOG_DEBUG_VERBOSE=0x8,
LOG_INIT=0x10,
LOG_GPS=0x20,
LOG_IMU=0x30,
LOG_LOG=0x40,
LOG_IO=0x50,
LOG_CONSOLE=0x60
};

void log_message(int log_level,const char* fmt,...);
