
enum
{
NO_FIX,
READY,
INIT,
ACTIVE,
};

bool log_start();
void log_stop();
bool log_is_running();
bool log_update();

