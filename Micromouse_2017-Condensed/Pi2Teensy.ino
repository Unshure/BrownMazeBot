#include <Wire.h>

#define BUFSIZE 256
#define SPEED_LIMIT 255
#define DEV_ID 1
#define RAMP_CONST 1 // Higher is faster

// Global vars
const int safesize = BUFSIZE / 2;
char buf[BUFSIZE];
char msg[BUFSIZE];
char write_buffer[safesize];
unsigned long msecs = 0;
unsigned long timeout = 0;
unsigned int encoder_reset = 0;

// Target and previous velocity arrays
static char action;
static int steps;

void serial()
{
  int available_bytes = 0;
  if ((available_bytes = Serial.available()))
  {
    // Read + attach null byte to read string
    int obytes = strlen(buf);
    Serial.readBytes(&buf[obytes], available_bytes);
    buf[available_bytes + obytes] = '\0';

    // Resize read string just in case
    if (strlen(buf) > safesize)
    {
      memmove(buf, &buf[strlen(buf) - safesize], safesize);
      buf[safesize] = '\0';
    }

    char *s, *e;

    if ((e = strchr(buf, '\n')))
    {
      e[0] = '\0';
      if ((s = strrchr(buf, '[')))
      {
        // Parse string being read
        // Left motor, right motor, reset encoders


        // Was originally:
        //sscanf(s, "[%d %d]\n", &target_vel[0], &target_vel[1]);
        //but for some reason this crashes code in 1.8.2, but not 1.6.8


        sscanf(s, "[%c %d]\n", &action, &steps);
        timeout = millis();
      }
      memmove(buf, &e[1], strlen(&e[1]) + sizeof(char));
    }
  }

//  // EMERGENCY STOP: MASTER COMM LOST
//  if (millis() - timeout > 500)
//  {
//    // After .5 seconds, stop the robot
//    memset(target_vel, 0, sizeof(int) * 2);
//    memset(prev_vel, 0, sizeof(int) * 2);
//    setmotors(0, 0);
//    timeout = millis();
//  }

  // Send back data over serial every 100ms
  if (millis() - msecs > 50)
  {
    for (int i = 0; i < 4; i++)
    {
    sprintf(write_buffer, "[%d %c %d]\n",
        DEV_ID,
        action,
        steps);
    Serial.print(write_buffer);
    msecs = millis();
    }
  }

}
