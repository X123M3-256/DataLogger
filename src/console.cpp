#include <stdarg.h>
#include <Arduino.h>
#include "console.h"
#include "gps.h"
#include "imu.h"
#include "calibration.h"
#include "log.h"
#include "io.h"

update_state_t update_states[4]={0};

void update_timer_start(int index)
{
update_state_t* state=update_states+index;

int update_time=micros();
state->update_start_time=update_time;
}

void update_timer_stop(int index)
{
update_state_t* state=update_states+index;

int update_time=micros();
state->update_delta=update_time-state->update_start_time;
state->total_update_delta+=state->update_delta;
	if(state->update_delta>state->longest_update_delta)state->longest_update_delta=state->update_delta;
state->update_count++;
}

#define MAX_PATH_LENGTH 256
char cur_dir[MAX_PATH_LENGTH]={'/',0};

void resolve_full_path(const char* path,char* full_path)
{
	if(path[0]=='/')
	{
	strcpy(full_path,path);
	}
	else
	{
	strcpy(full_path,cur_dir);
	int dst_pos=strlen(cur_dir);

	//TODO check bounds
	int src_pos=0;
		while(path[src_pos]!=0)
		{
		char path_segment[MAX_PATH_LENGTH];
		int seg_pos=0;	
			while(path[src_pos]!=0&&path[src_pos]!='/')
			{
			path_segment[seg_pos]=path[src_pos];
			seg_pos++;
			src_pos++;
			}
			if(path[src_pos]=='/')src_pos++;
		path_segment[seg_pos]=0;
			if(strcmp(".",path_segment)==0)continue;
			else if(strcmp("..",path_segment)==0)
			{
			dst_pos-=1;
				while(dst_pos>0)
				{
					if(full_path[dst_pos-1]=='/')
					{
					full_path[dst_pos]=0;
					break;
					}
				dst_pos--;
				}
			continue;
			}
		memcpy(full_path+dst_pos,path_segment,seg_pos);
		dst_pos+=seg_pos;
		full_path[dst_pos]='/';
		dst_pos++;
		full_path[dst_pos]=0;
		}
	}
}

void console_list_directory(const char* path)
{
char full_path[MAX_PATH_LENGTH];
resolve_full_path(path,full_path);

io_dir dir;
int err=dir.open(full_path);
	if(err)
	{
	usb_serial->writef("ls: \"%s\": %s\r\n",path,io_get_error_string(err));
	usb_serial->flush();
	return;
	}
char entry_path[MAX_PATH_LENGTH];
bool is_directory=false;
	while(dir.get_next_entry(entry_path,MAX_PATH_LENGTH,&is_directory))
	{
	usb_serial->write(entry_path);
	usb_serial->write("\r\n");
	}
usb_serial->flush();
dir.close();
}

void console_ls(char** argv,int argc)
{
	if(argc==0)console_list_directory("");
	else 
	{
		for(int i=0;i<argc;i++)
		{
		console_list_directory(argv[i]);
		}
	}
}

void console_cd(char** argv,int argc)
{

	if(argc==0)
	{
	cur_dir[0]='/';
	cur_dir[1]=0;
	}
	else if(argc==1)
	{
	char full_path[MAX_PATH_LENGTH];
	resolve_full_path(argv[0],full_path);
	
	io_dir dir;
	int err=dir.open(full_path);
		if(err)
		{
		usb_serial->writef("cd: \"%s\": %s\r\n",argv[0],io_get_error_string(err));
		usb_serial->flush();
		return;
		}
	dir.close();
	strncpy(cur_dir,full_path,MAX_PATH_LENGTH);
	}
	else usb_serial->write("cd: Too many arguments\r\n");
}

void console_cat(char** argv,int argc)
{
	if(argc==0)usb_serial->write("cat: Missing filename\r\n");
	else
	{
	io_file file;
		for(int i=0;i<argc;i++)
		{
		char full_path[MAX_PATH_LENGTH];
		resolve_full_path(argv[i],full_path);

		int err=file.open(full_path,O_READ_ONLY);
			if(err)
			{
			usb_serial->writef("cat: \"%s\": %s\r\n",argv[i],io_get_error_string(err));
			usb_serial->flush();
			break;
			}
		
			while(file.available()>0)
			{
			uint8_t byte=file.read_uint8();
				if(byte=='\n')usb_serial->write('\r');
			usb_serial->write(byte);
			}
		usb_serial->write("\r\n");
		usb_serial->flush();
		file.close();
		}
	}
}

void console_touch(char** argv,int argc)
{
	if(argc==0)usb_serial->write("touch: Missing filename\r\n");
	else
	{
	io_file file;
		for(int i=0;i<argc;i++)
		{
		char full_path[MAX_PATH_LENGTH];
		resolve_full_path(argv[i],full_path);

		int err=file.open(full_path,O_WRITE_ONLY|O_CREATE);
			if(err)
			{
			usb_serial->writef("touch: \"%s\": %s\r\n",argv[i],io_get_error_string(err));
			usb_serial->flush();
			break;
			}
		file.close();
		}
	}
}

void console_rm(char** argv,int argc)
{
	if(argc==0)usb_serial->write("rm: Missing filename\r\n");
	else
	{
	io_file file;
		for(int i=0;i<argc;i++)
		{
		char full_path[MAX_PATH_LENGTH];
		resolve_full_path(argv[i],full_path);

		int err=file.remove(full_path);
			if(err)
			{
				if(err==ERROR_UNSPECIFIED)err=ERROR_IS_DIRECTORY;//TODO this is a hack
			usb_serial->writef("rm: Cannot remove \"%s\": %s\r\n",argv[i],io_get_error_string(err));
			usb_serial->flush();
			break;
			}
		}
	}
}

void console_mkdir(char** argv,int argc)
{
	if(argc==0)usb_serial->write("mkdir: Missing directory name\r\n");
	else
	{
	io_dir dir;
		for(int i=0;i<argc;i++)
		{
		char full_path[MAX_PATH_LENGTH];
		resolve_full_path(argv[i],full_path);

		int err=dir.create(full_path);
			if(err)
			{
			usb_serial->writef("mkdir: Cannot create directory \"%s\" - %s\r\n",argv[i],io_get_error_string(err));
			usb_serial->flush();
			}
		}
	}

usb_serial->flush();
}

void console_rmdir(char** argv,int argc)
{
	if(argc==0)usb_serial->write("rmdir: Missing directory name\r\n");
	else
	{
	io_dir dir;
		for(int i=0;i<argc;i++)
		{
		char full_path[MAX_PATH_LENGTH];
		resolve_full_path(argv[i],full_path);

		int err=dir.remove(full_path);
			if(err)
			{
				if(err==ERROR_UNSPECIFIED)usb_serial->writef("rmdir: Cannot remove \"%s\": Directory not empty\r\n",full_path,io_get_error_string(err));//TODO this is a hack
				else usb_serial->writef("rmdir: Cannot remove \"%s\": %s\r\n",argv[i],io_get_error_string(err));
			usb_serial->flush();
			break;
			}
		}
	}
}



void console_mv(char** argv,int argc)
{

}

void console_imu_show()
{
//Clear input buffer
usb_serial->discard(0);

int first=1;
vector3_t gyro_integral=vector3(0,0,0);
	while(usb_serial->available()==0)
	{
	//Accumulate samples until we get a magnetometer reading
	int samples_loaded=0;
	imu_t samples[10];
	int ready=0;
	vector3_t accel=vector3(0,0,0);
	vector3_t gyro=vector3(0,0,0);
	vector3_t mag=vector3(0,0,0);
		while(!ready&&imu_get(samples+samples_loaded,1))
		{
		accel=vector3_add(accel,samples[samples_loaded].accel);
		gyro=vector3_add(gyro,samples[samples_loaded].gyro);
		gyro_integral=vector3_add(gyro_integral,vector3_scale(samples[samples_loaded].gyro,0.01));
			if(samples[samples_loaded].timestamp&SAMPLE_HAS_MAG)
			{
			mag=samples[samples_loaded].mag;
			ready=1;
			}
		samples_loaded++;
			if(samples_loaded==10)ready=1;
		}
		if(ready)
		{
		accel=vector3_scale(accel,1.0/samples_loaded);
		gyro=vector3_scale(gyro,180.0/(M_PI*samples_loaded));
		//Write output
			if(!first)usb_serial->write("\r\033[A\033[A\033[A\033[A\033[A\033[A");
		usb_serial->write("              |    X   |   Y   |   Z   |Magnitude|\r\n");
		usb_serial->writef("Accel (m/s^2) |% 7.2f % 7.2f % 7.2f |%6.2f   |\r\n",accel.x,accel.y,accel.z,vector3_magnitude(accel));
		usb_serial->writef("Gyro  (deg/s) |% 7.2f % 7.2f % 7.2f |%6.2f   |\r\n",gyro.x,gyro.y,gyro.z,vector3_magnitude(gyro));
		usb_serial->writef("Mag           |% 7.2f % 7.2f % 7.2f |%6.2f   |\r\n",mag.x,mag.y,mag.z,vector3_magnitude(mag));
		usb_serial->write("              |--------|-------|-------|---------|\r\n");
		//TODO this should display pitch/yaw/roll alongside the magnetometer derived orientation
		usb_serial->writef("Gyro sum (deg)|% 7.2f % 7.2f % 7.2f |%6.2f   |\r\n",180*gyro_integral.x/M_PI,180*gyro_integral.y/M_PI,180*gyro_integral.z/M_PI,180*vector3_magnitude(gyro_integral)/M_PI);
		usb_serial->flush();
		first=0;
		}
	}
}

void console_imu(char** argv,int argc)
{
	if(argc==0)
	{
	usb_serial->write("imu: No command given\r\n");
	return;
	}
	if(strcmp(argv[0],"show")==0)
	{
	console_imu_show();
	}
	else if(strcmp(argv[0],"calibration")==0)
	{
		if(argc==1)usb_serial->write("imu: calibration: Missing argument\r\n");
		else if(strcmp(argv[1],"reset")==0)imu_reset_calibration();
		else if(strcmp(argv[1],"start")==0)calibrate();
		else if(strcmp(argv[1],"load")==0)
		{
		const char* filename="calibration.txt";
			if(argc>=3)filename=argv[2];
			if(!imu_load_calibration(filename))usb_serial->writef("imu: calibration: Failed to load calibration \"%s\"\r\n",filename);
		}
	}
}

void console_gps_show()
{
//Clear any pending input
usb_serial->discard(0);

int lines=0;

	while(usb_serial->available()==0)
	{
	ubx_t ubx;
		while(ubx_receive(&ubx))
		{
			//UBX-NAV-PVT
			if(ubx.clss!=1||ubx.id!=7)continue;

		usb_serial->write('\r');
			while(lines>0)
			{
			usb_serial->write("\033[A");
			usb_serial->write("                                              \r");
			lines--;
			}
		usb_serial->writef("Timestamp     : %04d-%02d-%02d %02d:%02d:%02d.%02d\r\n",ubx.nav_pvt.year,ubx.nav_pvt.month,ubx.nav_pvt.day,ubx.nav_pvt.hour,ubx.nav_pvt.min,ubx.nav_pvt.sec,(int)round(ubx.nav_pvt.nanoseconds*1e-7));
		const char* fix_types[6]={"No fix","Dead reckoning","2D","3D","GNSS+Dead reckoning","Time only"};
		int fix_type=ubx.nav_pvt.fix_type;
		usb_serial->writef("Fix type      : %s\r\n",fix_types[fix_type]);
			if(fix_type>=2)
			{
		usb_serial->writef("Longitude     : %f deg\r\n",ubx.nav_pvt.longitude*1e-7);
		usb_serial->writef("Latitude      : %f deg\r\n",ubx.nav_pvt.latitude*1e-7);
		lines+=2;
			}
			if(fix_type>=3)
			{
		usb_serial->writef("Height        : %.1fm\r\n",ubx.nav_pvt.height*1e-3);
		usb_serial->writef("Altitude MSL  : %.1fm\r\n",ubx.nav_pvt.height_msl*1e-3);
		usb_serial->writef("Velocity N    : %.1fm/s\r\n",ubx.nav_pvt.velocity_n*1e-3);
		usb_serial->writef("Velocity E    : %.1fm/s\r\n",ubx.nav_pvt.velocity_e*1e-3);
		usb_serial->writef("Velocity D    : %.1fm/s\r\n",ubx.nav_pvt.velocity_d*1e-3);
		lines+=5;
			}
			if(fix_type>=2)
			{
		usb_serial->writef("Accuracy Horz : %.2fm\r\n",ubx.nav_pvt.horizontal_acc*1e-3);
		lines+=1;
			}
			if(fix_type>=3)
			{
		usb_serial->writef("Accuracy Vert : %.2fm\r\n",ubx.nav_pvt.vertical_acc*1e-3);
		lines+=1;
			}
			if(fix_type>=2)
			{
		usb_serial->writef("Accuracy speed: %.2fm/s\r\n",ubx.nav_pvt.speed_acc*1e-3);
		usb_serial->writef("Position DOP  : %.2fm/s\r\n",ubx.nav_pvt.speed_acc*1e-2);
		lines+=2;
			}
		usb_serial->writef("Satellites    : %d\r\n",ubx.nav_pvt.num_sv);
		lines+=3;	
		usb_serial->flush();
		}

	}
}

void console_gps(char** argv,int argc)
{
	if(argc==0)
	{
	usb_serial->write("gps: No command given\r\n");
	return;
	}
	if(strcmp(argv[0],"show")==0)
	{
	console_gps_show();
	}
}


void console_log(char** argv,int argc)
{
	if(argc==0)
	{
	usb_serial->write("log: No command given\r\n");
	return;
	}
	if(strcmp(argv[0],"start")==0)log_start();
	else if(strcmp(argv[0],"stop")==0)log_stop();
}

void console_ups_update(int index)
{
update_state_t* state=update_states+index;
usb_serial->writef("Last update time: %.3lfms\r\n",0.001*state->update_delta);
usb_serial->writef("Longest update time: %.3fms\r\n",0.001*state->longest_update_delta);
double av=(state->total_update_delta)/(double)state->update_count;
usb_serial->writef("Average update time: %.3fms\r\n",0.001*av);
}

void console_ups(char** argv,int argc)
{
usb_serial->write("Log:\r\n");
console_ups_update(UPDATE_LOG);
usb_serial->write("Console:\r\n");
console_ups_update(UPDATE_CONSOLE);
usb_serial->write("Total:\r\n");
console_ups_update(UPDATE_TOTAL);
usb_serial->flush();
}





/*
#define SUCCEED(num) {*out=num;return pos;}
#define FAIL() {return 0;}
//TODO distinguish between definitely failed, and could potentially succeed if more input was supplied

int8_t digit_value(uint8_t byte)
{
	if(byte>='0'&&byte<='9')return byte-'0';
	else if(byte>='a'&&byte<='z')return byte-'a'+10;
	else if(byte>='A'&&byte<='Z')return byte-'A'+10;
return -1;
}


size_t parse_func_uint32(int file,size_t pos,uint32_t* out)
{
size_t len_max=34+pos;
uint8_t buf[len_max];//longest valid value would be 32 bit binary constant
size_t len=peek(file,buf,len_max);//TODO implement peek_offset
	if(len==0)FAIL();

//Parse base specifier
int base=10;
	if(buf[pos]=='0')
	{
		if(len==1)SUCCEED(0);
	pos++;
		if(buf[pos]=='x'){base=16;pos++;}
		else if(buf[pos]=='b'){base=2;pos++;}
		else if(buf[pos]>='0'&&buf[pos]<='9')base=8;
		else SUCCEED(0);
	}

//Parse digits
int32_t val=0;
bool valid=false;
	while(pos<len)
	{
	int32_t digit=digit_value(buf[pos]);
		if(digit<0||digit>=base)break;
	Serial.println(digit);
	val=val*base+digit;
	valid=true;
	pos++;
	}
	if(!valid)FAIL();
//TODO check range
SUCCEED(val);
}

size_t parse_func_int32(int file,size_t pos,int32_t* out)
{
uint8_t byte;
	if(peek(file,&byte,1)==0)FAIL();

//Parse negative sign
bool neg=false;
	if(byte=='-')
	{
	neg=true;
	pos++;
	}

//Parse digits
uint32_t val=0;
size_t len=parse_func_uint32(file,pos,&val);
	if(len==0)FAIL();
pos+=len;

//TODO check range
	if(neg)val*=-1;
SUCCEED((int32_t)val);
}


bool parse(int file,size_t (*parse_func)(int file,size_t pos,void* out),void* out)
{
size_t len=parse_func(file,0,out);


	if(len!=0)
	{
	discard(file,len);
	return true;
	}
return false;
}

bool parse_int32(int file,int32_t* out)
{
return parse(file,parse_func_int32,out);
}


void console_try_parse(const char* buf,int len)
{
//Write entered string to file
int file=open_file("test.txt",O_WRITE_ONLY);
	if(file<0)
	{
	usb_serial->write("Failed to open test.txt for writing\r\n");
	usb_serial->flush();
	return;
	}
write(file,buf,len);
close_file(file);


file=open_file("test.txt",O_READ_ONLY);
	if(file<0)
	{
	usb_serial->write("Failed to open test.txt for reading\r\n");
	usb_serial->flush();
	return;
	}

int32_t val;
	if(parse_int32(file,&val))log_message(LOG_CONSOLE|LOG_INFO,"Parsed int32 %d (0x%x)\n",val,val);
	else 
	{
	log_message(LOG_CONSOLE|LOG_INFO,"Parse failed\n",val);
	return;
	}
;

close_file(file);
}
*/
void console_execute_command(const char* buf,int len)
{
//TODO check these bounds
char cmd[512];
char arg[4][512];
int arg_count=0;
	if(len==0)return;
//Parse command
int i=0;
	while(buf[i]!=' '&&i<len)
	{
	cmd[i]=buf[i];
	i++;
	}
cmd[i]=0;
//Parse arguments
	while(i<len)
	{
	//Skip whitespace
		while(buf[i]==' '&&i<len)i++;
		if(i==len)break;
	//Parse argument
	int j=0;
		while(buf[i]!=' '&&i<len)
		{
		arg[arg_count][j]=buf[i];
		i++;
		j++;
		}
	arg[arg_count][j]=0;
	arg_count++;
	}
char* argv[4];
	for(int i=0;i<arg_count;i++)argv[i]=arg[i];

	if(strcmp(cmd,"ls")==0)console_ls(argv,arg_count);
	else if(strcmp(cmd,"cd")==0)console_cd(argv,arg_count);
	else if(strcmp(cmd,"cat")==0)console_cat(argv,arg_count);
	else if(strcmp(cmd,"touch")==0)console_touch(argv,arg_count);
	else if(strcmp(cmd,"rm")==0)console_rm(argv,arg_count);
	else if(strcmp(cmd,"mkdir")==0)console_mkdir(argv,arg_count);
	else if(strcmp(cmd,"rmdir")==0)console_rmdir(argv,arg_count);
	else if(strcmp(cmd,"mv")==0)console_mv(argv,arg_count);
	else if(strcmp(cmd,"imu")==0)console_imu(argv,arg_count);
	else if(strcmp(cmd,"gps")==0)console_gps(argv,arg_count);
	else if(strcmp(cmd,"log")==0)console_log(argv,arg_count);
	else if(strcmp(cmd,"ups")==0)console_ups(argv,arg_count);
	else
	{
	//console_try_parse(buf,len);
	usb_serial->writef("console: Unrecognized command \"%s\"\r\n");
	usb_serial->flush();
	}
}




bool console_active=false;
#define INPUT_BUFFER_LENGTH 512
char input[INPUT_BUFFER_LENGTH];
int input_length=0;
int last_input_length=0;
int cursor_pos=0;

enum
{
ESCAPE_LEFT_ARROW,
ESCAPE_RIGHT_ARROW,
ESCAPE_DELETE,
ESCAPE_COUNT,
ESCAPE_NO_MATCH,
ESCAPE_NO_MATCH_YET
};


//TODO show current directory
void console_show_prompt()
{
usb_serial->write("\r");
int cur_dir_len=strlen(cur_dir)-1;
	if(cur_dir_len<0)cur_dir_len=0;

	if(cur_dir_len>0)
	{
	usb_serial->write(cur_dir+1,cur_dir_len);
	}
usb_serial->write("$ ");


usb_serial->write(input,input_length);
if(last_input_length>input_length)
{
for(int i=input_length;i<last_input_length;i++)usb_serial->write(' ');
for(int i=cursor_pos;i<last_input_length;i++)usb_serial->write("\033[D");
}
else
{
		for(int i=cursor_pos;i<input_length;i++)usb_serial->write("\033[D");
	}
usb_serial->flush();
}

int match_escape_code(uint8_t* bytes,size_t num_bytes)
{
const char* escape_codes[ESCAPE_COUNT]={"[D","[C","[3~"};
bool have_candidate=false;
	for(int i=0;i<ESCAPE_COUNT;i++)
	{
	size_t len=strlen(escape_codes[i]);
		if(memcmp(bytes+1,escape_codes[i],num_bytes-1)==0)
		{
			//We have matched the complete code
			if(len+1==num_bytes)return i;
		//All the bytes read so far match
		have_candidate=true;
		}
	}
	if(!have_candidate)return ESCAPE_NO_MATCH;
return ESCAPE_NO_MATCH_YET;
}

bool console_update()
{
	if(!console_active)
	{
		//Start console if the USB usb_serial port is available
		if(usb_serial->open(9600,O_READ_WRITE))
		{
		////TODO dump contents of debug log once that exists
		console_active=true;
		console_show_prompt();
		}//TODO stop running the console if the USB usb_serial port is disconnected
	
	return false;
	}

int flags=0;
last_input_length=input_length;
	while(usb_serial->available()>0&&!(flags&2))
	{
	uint8_t bytes[4];
	size_t num_bytes=usb_serial->peek(bytes,4);
		//I don't think this can happen but we check for it just in case
		if(num_bytes==0)
		{
		log_message(LOG_INIT|LOG_DEBUG,"peek() returned 0 bytes");
		break;
		}
		//Check for escape sequences
		if(bytes[0]==0x1B)
		{
		int escape_code=match_escape_code(bytes,num_bytes);
			switch(escape_code)
			{
			case ESCAPE_LEFT_ARROW:
					if(cursor_pos>0)
					{
					cursor_pos--;
					flags|=1;
					}
				usb_serial->discard(3);
			break;
			case ESCAPE_RIGHT_ARROW:
					if(cursor_pos<input_length)
					{
					cursor_pos++;
					flags|=1;
					}
				usb_serial->discard(3);
			break;
			case ESCAPE_DELETE:
					if(cursor_pos<input_length)
					{
					input_length--;
					memmove(input+cursor_pos,input+cursor_pos+1,input_length-cursor_pos);
					flags|=1;
					}
				usb_serial->discard(4);
			break;
			case ESCAPE_NO_MATCH:
			//If nothing matches, discard the escape character
			usb_serial->discard(1);
			break;
			}
		continue;
		}
		//Printable character
		if(bytes[0]>=' '&&bytes[0]<='~')
		{
			if(input_length<INPUT_BUFFER_LENGTH)
			{
			memmove(input+cursor_pos+1,input+cursor_pos,input_length-cursor_pos);
			input[cursor_pos]=bytes[0];
			cursor_pos++;
			input_length++;
			flags|=1;
			}
		}
		//Backspace
		else if(bytes[0]==8)
		{
			if(cursor_pos>0)
			{
			cursor_pos--;
			input_length--;
			memmove(input+cursor_pos,input+cursor_pos+1,input_length-cursor_pos);
			flags|=1;
			}
		}
		//Line break
		else if(bytes[0]=='\r'||bytes[0]=='\r')
		{
		flags|=2;
		}
	usb_serial->discard(1);
	}
//If we recieved a line break, the command is complete and should be executed
	if(flags&2)
	{
	usb_serial->write("\r\n");
	console_execute_command(input,input_length);
	input_length=0;
	cursor_pos=0;
	}
//If the input buffer was modified, the prompt now needs to be redrawn
	if(flags)console_show_prompt();
return flags!=0;
}

