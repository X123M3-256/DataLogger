typedef struct
{
long int update_start_time=0;
long int update_count=0;
long int update_delta=0;
long int total_update_delta=0;
long int longest_update_delta=0;
}update_state_t;

enum
{
UPDATE_GPS=0,
UPDATE_LOG=1,
UPDATE_CONSOLE=2,
UPDATE_TOTAL=3,
};

//Used for ups command
void update_timer_start(int index);
void update_timer_stop(int index);

int console_init();
void console_run_command(int command);
bool console_update();
