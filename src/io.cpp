#include <Arduino.h>
#include "timer.h"
#include "io.h"

typedef struct
{
uint8_t* bytes;
size_t len;
}buf_info_t;

size_t ring_buffer_copy_src_func(uint8_t* bytes,size_t len,void* data)
{
buf_info_t* src=(buf_info_t*)data;
	if(len>src->len)len=src->len;
memcpy(bytes,src->bytes,len);
src->bytes+=len;
src->len-=len;
return len;
}

size_t ring_buffer_copy_dst_func(uint8_t* bytes,size_t len,void* data)
{
buf_info_t* dst=(buf_info_t*)data;
	if(len>dst->len)len=dst->len;
memcpy(dst->bytes,bytes,len);
dst->bytes+=len;
dst->len-=len;
return len;
}



io_stream::io_stream()
{
flags=0;
read_buffer=ring_buffer<uint8_t,READ_BUFFER_SIZE>();
write_buffer=ring_buffer<uint8_t,WRITE_BUFFER_SIZE>();
}

bool io_stream::is_writeable()
{
return flags&IO_WRITE_ENABLE;
}
bool io_stream::is_readable()
{
return flags&IO_READ_ENABLE;
}

size_t io_stream::available()
{
return read_buffer.available()+available_raw();
}

size_t read_buf_func(uint8_t* buf,size_t len,void* data)
{
io_stream* target=(io_stream*)data;
size_t n=target->read_raw(buf,len);
return n;
}

size_t io_stream::read(uint8_t* buf,size_t len)
{
	//Check if target is readable
	if(!(flags&IO_READ_ENABLE))
	{
	log_message(LOG_IO|LOG_ERROR,"read: Stream is write-only");
	delay(1000);
	return 0;
	}

	//Check if input buffering is disabled
	if(!(flags&IO_READ_BUFFER))return read_raw((uint8_t*)(buf),len);

//Serial.println("read: Buffer available");
//Serial.println(read_buffer.available());

//Read data from buffer
buf_info_t buf_inf={buf,len};
	if(len==0)return 0;
size_t bytes_read=read_buffer.read(ring_buffer_copy_dst_func,(void*)(&buf_inf),len);


//Serial.println("read: Read from buffer");
//Serial.println(bytes_read);

size_t source_available=available_raw();
	//If read buffer is empty and there is more data available, read it in
	if(read_buffer.available()==0&&source_available>0)
	{
	size_t remaining_bytes=len-bytes_read;

	//Serial.println("read: Source available");
	//Serial.println(source_available);

	//Serial.println("read: Remaining");
	//Serial.println(remaining_bytes);
	//If the source has fewer bytes available than we want to read - or if we want to read more bytes
	//than will fit in the buffer, the data read into the buffer would be immediately read out again.
	//So to avoid unnecessary copying, we copy the data directly to the output.
		if(source_available<=remaining_bytes&&remaining_bytes>=read_buffer.capacity())bytes_read+=read_raw(buf+bytes_read,remaining_bytes);
		else
		{
		//Read all available data into buffer
		read_buffer.write(read_buf_func,(void*)(this),source_available);
		//Copy requested data to outputa
			if(remaining_bytes>0)
			{
			buf_info_t buf_inf={buf+bytes_read,len};
			bytes_read+=read_buffer.read(ring_buffer_copy_dst_func,(void*)(&buf_inf),remaining_bytes);
			}
		}
	}
return bytes_read;
}

size_t io_stream::peek(uint8_t* buf,size_t len)
{
	//Check if target_num can be read
	if(!(flags&IO_READ_ENABLE))
	{
	log_message(LOG_IO|LOG_ERROR,"peek: Stream is write-only");
	delay(1000);
	return 0;
	}

	//If input buffering is disabled, fail
	//TODO When seeking, is implemented, peek could be implemented by reading data and
	//then seeking back for targets which support it. Some targets also have an underlying
	//peek() method which could be used.
	if(!(flags&IO_READ_BUFFER))
	{
	log_message(LOG_IO|LOG_ERROR,"peek: Stream has no input buffering");
	delay(1000);
	return 0;
	}
//If more data was requested than is currently available in the buffer, we read all available data from the source
//into the buffer
size_t source_available=available_raw();
	if(len>read_buffer.available()&&source_available>0)
	{
	//read_buffer.write(read_buf_func,this,len-read_buffer.available());
	read_buffer.write(read_buf_func,this,0);
	}
return read_buffer.peek(buf,len);
}

size_t io_stream::discard(size_t len)
{
	//Check if target_num can be read
	if(!(flags&IO_READ_ENABLE))
	{
	log_message(LOG_IO|LOG_ERROR,"discard: Stream is write-only");
	delay(1000);
	return 0;
	}

	//TODO discard could be implemented for non-buffered I/O but it'd be quite slow and not really useful
	//unless peek is also implemented.
	if(!(flags&IO_READ_BUFFER))return 0;

//Discard data in the buffer
size_t bytes_discarded=read_buffer.discard(len);
	while(bytes_discarded<len&&available_raw()>0)
	{
	read_buffer.write(read_buf_func,this,0);
	bytes_discarded+=read_buffer.discard(len);
	}
return bytes_discarded;
}

size_t write_buf_func(uint8_t* buf,size_t len,void* data)
{
io_stream* target=(io_stream*)data;
return target->write_raw(buf,len);
}


size_t io_stream::write(uint8_t* buf,size_t len)
{
	//Check if target_num can be written
	if(!(flags&IO_WRITE_ENABLE))
	{
	log_message(LOG_IO|LOG_DEBUG,"write : Stream is read-only");
	delay(1000);
	return 0;
	}

	//Check if output buffering is disabled
	if(!(flags&IO_WRITE_BUFFER))return write_raw((uint8_t*)(buf),len);


buf_info_t buf_inf={buf,len};
size_t bytes_written=write_buffer.write(ring_buffer_copy_src_func,(void*)(&buf_inf),len);

	//If buffer is full, flush it now
	if(write_buffer.free()==0)flush();

//If all data has been written, return immediately
	if(bytes_written==len)return bytes_written;

//If the number of bytes still to be written exceeds the size of the buffer, there's no point writing that
//data into the buffer just to flush it out again. So in that case, we write that data directly to the output
size_t remaining_bytes=len-bytes_written;
	if(remaining_bytes>write_buffer.capacity())bytes_written+=write_raw((uint8_t*)(buf+bytes_written),remaining_bytes);
	else
	{
	buf_inf={buf+bytes_written,remaining_bytes};
	bytes_written+=write_buffer.write(ring_buffer_copy_src_func,(void*)(&buf_inf),len);
	}

return bytes_written;
}

void io_stream::flush()
{
	if(flags&IO_WRITE_BUFFER)
	{
	//TODO make sure this really does always clear the write buffer
	write_buffer.read(write_buf_func,this,0);
	}
flush_raw();
}


size_t io_stream::write(const char* buf,size_t len)
{
return write((uint8_t*)buf,len);
}

size_t io_stream::write(const char* buf)
{
return write((uint8_t*)buf,strlen(buf));
}

size_t io_stream::write(const char c)
{
return write((uint8_t*)(&c),1);
}

size_t io_stream::vwritef(const char* fmt,va_list args)
{
int len=vsnprintf(NULL,0,fmt,args);
char str[len+1];
vsnprintf(str,len+1,fmt,args);
return write((uint8_t*)str,len);
}

size_t io_stream::writef(const char* fmt,...)
{
va_list args;
va_start(args,fmt);
size_t bytes_written=vwritef(fmt,args);
va_end(args);
return bytes_written;
}


uint8_t io_stream::read_uint8()
{
uint8_t res;
read(&res,sizeof(uint8_t));
return res;
}
uint16_t io_stream::read_uint16()
{
uint16_t res;
read((uint8_t*)(&res),sizeof(uint16_t));
return res;
}
uint32_t io_stream::read_uint32()
{
uint32_t res;
read((uint8_t*)(&res),sizeof(uint32_t));
return res;
}
int8_t io_stream::read_int8()
{
int8_t res;
read((uint8_t*)(&res),sizeof(int8_t));
return res;
}
int16_t io_stream::read_int16()
{
int16_t res;
read((uint8_t*)(&res),sizeof(int16_t));
return res;
}
int32_t io_stream::read_int32()
{
int32_t res;
read((uint8_t*)(&res),sizeof(int32_t));
return res;
}
float io_stream::read_float()
{
float res;
read((uint8_t*)(&res),sizeof(float));
return res;
}
double io_stream::read_double()
{
double res;
read((uint8_t*)(&res),sizeof(double));
return res;
}

size_t read_while(bool (*f)(uint8_t),uint8_t* buf,size_t len)
{
size_t bytes_written=0;
	while(available()>0&&bytes_written<len)
	{
	uint8_t byte=peek_uint8();
		if(!f(byte))return bytes_written;
	buf[bytes_written]=byte;
	bytes_written++;
	discard(1);
	}
return bytes_written;
}

size_t read_until(bool (*f)(uint8_t),uint8_t* buf,size_t len)
{
size_t bytes_written=0;
	while(available()>0&&bytes_written<len)
	{
	uint8_t byte=peek_uint8();
		if(f(byte))return bytes_written;
	buf[bytes_written]=byte;
	bytes_written++;
	discard(1);
	}
return bytes_written;
}





bool io_serial::open(uint32_t baud,uint32_t open_flags)
{
log_message(LOG_IO|LOG_DEBUG_VERBOSE,"io_serial: Starting serial %d with baud rate %d and mode 0x%x",id(),baud,open_flags);

flags=open_flags|IO_READ_BUFFER|IO_WRITE_BUFFER;
	
	if(begin(baud))return true;

log_message(LOG_IO|LOG_DEBUG_VERBOSE,"io_serial: Failed to start serial %d",id());
return false;
}

void io_serial::close()
{
end();
timer_wait_for_lock(2);
Serial.end();
timer_unlock();
flags&=~IO_OPEN;
flags&=~IO_READ_ENABLE;
flags&=~IO_WRITE_ENABLE;
}

class io_usb_serial:public io_serial
{
public:
	virtual uint32_t id();
	virtual bool begin(uint32_t baud);
	virtual void end();
	virtual size_t write_raw(const uint8_t*,size_t);
	virtual size_t read_raw(uint8_t*,size_t);
	virtual size_t available_raw();
	virtual void flush_raw();
};

uint32_t io_usb_serial::id()
{
return 0;
}


bool io_usb_serial::begin(uint32_t baud)
{
timer_wait_for_lock(2);
Serial.begin(baud);
	if(Serial)
	{
	flags|=IO_OPEN;
	timer_unlock();
	return true;
	}
timer_unlock();
return false;
}

void io_usb_serial::end()
{
timer_wait_for_lock(2);
Serial.end();
timer_unlock();
}

size_t io_usb_serial::read_raw(uint8_t* buf,size_t len)
{
timer_wait_for_lock(2);
size_t avail=Serial.available();
	if(avail<len)len=avail;
size_t bytes=Serial.readBytes((char*)buf,len);
timer_unlock();
return bytes;
}
size_t io_usb_serial::write_raw(const uint8_t* buf,size_t len)
{
timer_wait_for_lock(2);
size_t bytes=Serial.write((char*)buf,len);
timer_unlock();
return bytes;
}
size_t io_usb_serial::available_raw()
{
timer_wait_for_lock(2);
size_t bytes=Serial.available();
timer_unlock();
return bytes;
}

void io_usb_serial::flush_raw()
{
timer_wait_for_lock(2);
Serial.flush();
timer_unlock();
}

class io_hardware_serial:public io_serial
{
public:
	virtual uint32_t id();
	virtual bool begin(uint32_t baud);
	virtual void end();
	virtual size_t write_raw(const uint8_t*,size_t);
	virtual size_t read_raw(uint8_t*,size_t);
	virtual size_t available_raw();
	virtual void flush_raw();
};


uint32_t io_hardware_serial::id()
{
return 1;
}

bool io_hardware_serial::begin(uint32_t baud)
{
timer_wait_for_lock(2);
Serial1.begin(baud);
	if(Serial1)
	{
	flags|=IO_OPEN;
	timer_unlock();
	return true;
	}
timer_unlock();
return false;
}



void io_hardware_serial::end()
{
timer_wait_for_lock(2);
Serial1.end();
timer_unlock();
}

size_t io_hardware_serial::read_raw(uint8_t* buf,size_t len)
{
timer_wait_for_lock(2);
size_t avail=Serial1.available();
	if(avail<len)len=avail;
size_t bytes=Serial1.readBytes((char*)buf,len);
timer_unlock();
//usb_serial->writef("io_hardware_serial: Requested %d bytes read %d\r\n",len,bytes);
return bytes;
}

size_t io_hardware_serial::write_raw(const uint8_t* buf,size_t len)
{
timer_wait_for_lock(2);
size_t bytes=Serial1.write((char*)buf,len);
timer_unlock();
return bytes;
}

size_t io_hardware_serial::available_raw()
{


timer_wait_for_lock(2);
size_t bytes=Serial1.available();
timer_unlock();
return bytes;
}

void io_hardware_serial::flush_raw()
{
timer_wait_for_lock(2);
Serial1.flush();
timer_unlock();
}


SdFat32 sd;

const char* io_get_error_string(int err)
{
const char* strs[8]={"No error","No such file or directory","Is a directory","Must specify one of O_READ_WRITE, O_READ_ONLY, O_WRITE_ONLY","Path not set","Already exists","Is a file","Unspecified error"};
return strs[err];
}

io_file::io_file()
{
path=NULL;
}
io_file::io_file(const char* name)
{
path=name;
}
	
int io_file::open(uint32_t mode)
{
	if(flags&IO_OPEN)close();//TODO should this be an error

log_message(LOG_IO|LOG_DEBUG_VERBOSE,"open_file: Opening file %s",path);

	if(path==NULL)return ERROR_PATH_NOT_SET;

flags=mode;

//Build SdFat open mode
uint32_t sd_mode=0;
	if((mode&O_READ_WRITE)==O_READ_WRITE)sd_mode=O_RDWR;
	else if(mode&O_READ_ONLY)sd_mode=O_READ;
	else if(mode&O_WRITE_ONLY)sd_mode=O_WRITE|O_TRUNC;
	else
	{
	log_message(LOG_IO|LOG_DEBUG,"open_file: Must specify one of O_READ_WRITE, O_READ_ONLY, O_WRITE_ONLY");
	return ERROR_INVALID_MODE;
	}
	//TODO fail if the file is read only
	if(mode&O_CREATE)sd_mode|=O_CREAT;
log_message(LOG_IO|LOG_DEBUG_VERBOSE,"open_file: Using SdFat open mode %x (my mode %x)",sd_mode,mode);

timer_wait_for_lock(2);
//Check that target file exists
	if(!sd.exists(path)&&!(mode&&O_CREATE))
	{
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"open_file: Cannot open \"%s\" - no such file or directory",path);
	return ERROR_DOES_NOT_EXIST;
	}
//Attempt to open the file
file.open(path,sd_mode);
	if(!file)
	{
	//TODO This really needs a more specific error message
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"open_file: Failed to open file");
	return ERROR_UNSPECIFIED;
	}

	if(file.isDir())
	{
	file.close();
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"open_file: \"%s\" is a directory",path);
	return ERROR_IS_DIRECTORY;
	}
timer_unlock();
flags|=IO_OPEN;
return 0;
}
int io_file::open(const char* name,uint32_t mode)
{
path=name;
return open(mode);
}
void io_file::close()
{
	if(!(flags&IO_OPEN))
	{
	log_message(LOG_IO|LOG_DEBUG,"close_file: File is not open");
	return;
	}
//Flush output buffer
flush();

timer_wait_for_lock(2);
file.close();
timer_unlock();
flags&=~IO_OPEN;
}

int io_file::remove()
{
	if(!(flags&IO_OPEN))
	{
	int err=open(O_WRITE_ONLY);//TODO should this be an error
		if(err!=0)return err;
	}

timer_wait_for_lock(2);
	if(!file.remove())
	{
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"io_file::remove: Failed to remove file");
	return ERROR_UNSPECIFIED;
	}
timer_unlock();
close();

return 0;
}
int io_file::remove(const char* name)
{
path=name;
return remove();
}

size_t io_file::read_raw(uint8_t* buf,size_t len)
{
return file.read(buf,len);
}
size_t io_file::write_raw(const uint8_t* buf,size_t len)
{
return file.write(buf,len);
}
size_t io_file::available_raw()
{
return file.available();
}
void io_file::flush_raw()
{
return file.flush();
}



io_dir::io_dir()
{
path=NULL;
flags=0;
}
io_dir::io_dir(const char* name)
{
path=name;
flags=0;
}

int io_dir::open()
{
	if(flags&IO_OPEN)close();//TODO should this be an error

log_message(LOG_IO|LOG_DEBUG_VERBOSE,"open_dir: Opening dir %s",path);

	if(path==NULL)return ERROR_PATH_NOT_SET;

timer_wait_for_lock(2);
//Check that target dir exists
	if(!sd.exists(path))
	{
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"open_dir: Cannot open \"%s\" - no such file or directory",path);
	return ERROR_DOES_NOT_EXIST;
	}

//Attempt to open the dir
dir.open(path,O_READ);
	if(!dir)
	{
	//TODO This really needs a more specific error message
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"open_dir: Failed to open dir");
	return ERROR_UNSPECIFIED;
	}

	if(dir.isFile())
	{
	dir.close();
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"open_dir: \"%s\" is a file",path);
	return ERROR_IS_FILE;
	}
timer_unlock();
flags|=IO_OPEN;
return 0;
}
int io_dir::open(const char* name)
{
path=name;
return open();
}

int io_dir::create()
{
	if(flags&IO_OPEN)close();//TODO should this be an error

log_message(LOG_IO|LOG_DEBUG,"io_dir::create: Creating directory \"%s\"",path);


timer_wait_for_lock(2);
	if(sd.exists(path))
	{
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"io_dir::create: Cannot create directory \"%s\" - already exists",path);
	return ERROR_ALREADY_EXISTS;
	}

	if(!sd.mkdir(path))
	{
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"io_dir::create: Failed to create directory");
	return ERROR_UNSPECIFIED;
	}
timer_unlock();
return 0;
}
int io_dir::create(const char* name)
{
path=name;
return create();
}
void io_dir::close()
{
	if(!(flags&IO_OPEN))
	{
	log_message(LOG_IO|LOG_DEBUG,"close_dir: File is not open");
	return;
	}
timer_wait_for_lock(2);
dir.close();
timer_unlock();
flags&=~IO_OPEN;
}

int io_dir::remove()
{
	if(!(flags&IO_OPEN))//TODO should this be an error
	{
	int err=open();
		if(err!=0)return err;
	}
timer_wait_for_lock(2);
	if(!dir.rmdir())
	{
	timer_unlock();
	log_message(LOG_IO|LOG_DEBUG,"io_dir::remove: Failed to remove dir");
	return ERROR_UNSPECIFIED;
	}
timer_unlock();
close();

return 0;
}
int io_dir::remove(const char* name)
{
path=name;
return remove();
}

bool io_dir::get_next_entry(char* name,int len,bool* is_directory)
{
SdFile file;
	if(file.openNext(&dir,O_READ))
	{
	file.getName(name,len);
		if(file.isDir())*is_directory=true;
		else *is_directory=false;
	file.close();
	return true;
	}
return false;
}

class io_test:public io_stream
{
public:
	void begin(int N);
	virtual size_t write_raw(const uint8_t*,size_t);
	virtual size_t read_raw(uint8_t*,size_t);
	virtual size_t available_raw();
	virtual void flush_raw();
private:
int N=1000;
int n=0;
};
void io_test::begin(int max)
{
N=max;
flags|=IO_READ_ENABLE;
flags|=IO_READ_BUFFER;
flags|=IO_WRITE_ENABLE;
flags|=IO_WRITE_BUFFER;
}
size_t io_test::read_raw(uint8_t* buf,size_t len)
{
	for(size_t i=0;i<len;i++)
	{
	buf[i]=n;
	n++;
	}
usb_serial->flush();
delay(10);
return len;
}
size_t io_test::write_raw(const uint8_t* buf,size_t len)
{
	for(size_t i=0;i<len;i++)
	{
	int val=buf[i];
	char str[256];
		if(i==len-1)sprintf(str,"%3d\r\n",val);
		else sprintf(str,"%3d ",val);
	Serial.print(str);
	}
Serial.println();
return len;
}
size_t io_test::available_raw()
{
return N-n;
}
void io_test::flush_raw()
{
return;
}
void io_test_read()
{
	while(!usb_serial->open(9600,O_READ_WRITE))
	{
	log_message(LOG_INIT|LOG_INFO,"Waiting for serial");
	delay(500);
	}

io_test test;
test.begin(10000);

	while(test.available())
	{
	uint8_t buf[30];
	size_t len=test.read(buf,30);
	Serial.print(len);
		for(size_t i=0;i<len;i++)
		{
		usb_serial->writef("%3d ",buf[i]);
		}
	usb_serial->write("\r\n");
	usb_serial->flush();
	delay(100);
	}
}



io_serial* usb_serial;
io_serial* serial_ports[2];

bool io_init()
{
usb_serial=new io_usb_serial();
serial_ports[0]=usb_serial;
serial_ports[1]=new io_hardware_serial();

//We have serial port
	if(!sd.begin(SdioConfig(FIFO_SDIO)))
	{
	log_message(LOG_IO|LOG_ERROR,"Failed to initialize SD card");
	return false;
	}
return true;
}





//log level for
int serial_log_mask=0x3;
int file_log_mask=0xF;

void log_message(int log_meta,const char* fmt,...)
{
	if(!usb_serial->is_writeable())return;

va_list args;
va_start(args,fmt);

int subsystem=log_meta&0xF0;

//If no log level is specified, treat as INFO
int log_level=LOG_INFO;
	if(log_meta&0x8)log_level=LOG_DEBUG_VERBOSE;
	else if(log_meta&0x4)log_level=LOG_DEBUG;
	else if(log_meta&0x2)log_level=LOG_ERROR;
	
	if(log_level&serial_log_mask)
	{
		switch(subsystem)
		{
		case LOG_INIT:
		usb_serial->write("init: ");
		break;
		case LOG_IO:
		usb_serial->write("io: ");
		break;
		case LOG_GPS:
		usb_serial->write("gps: ");
		break;
		case LOG_IMU:
		usb_serial->write("imu: ");
		break;
		case LOG_LOG:
		usb_serial->write("log: ");
		break;
		case LOG_CONSOLE:
		usb_serial->write("console: ");
		break;
		default:
		break;
		}

		if(log_meta&0x2)usb_serial->write("error: ");
		else if(log_meta&0xC)usb_serial->write("debug: ");

	usb_serial->vwritef(fmt,args);
	usb_serial->write("\r\n");
	}
usb_serial->flush();
va_end(args);
}
