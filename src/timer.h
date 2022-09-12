
void timer_start();
uint64_t timer_now();
void timer_set_gps_time(uint64_t timestamp);


uint64_t timer_now();
bool timer_lock();
void timer_unlock();
void timer_wait_for_lock(int millis_required=0);
