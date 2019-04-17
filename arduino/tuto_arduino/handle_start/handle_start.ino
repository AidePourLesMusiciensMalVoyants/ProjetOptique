
long speed = 9600;
//long speed = 115200;


void setup() {
  Serial.begin(speed); // use the same baud-rate as the python side
}

char buffer[256];
int pos = 0;

long number = 0;

int command()
{
  if(Serial.available() > 0) {
    char data = Serial.read();
    if (data == '#')
    {
      if (strcmp(buffer, "start") == 0)
      {
        pos = 0;
        return 1;
      }
      if (strcmp(buffer, "stop") == 0)
      {
        pos = 0;
        return 2;
      }
      if (strcmp(buffer, "running") == 0)
      {
        pos = 0;
        return 3;
      }

      // Serial.flush();

      pos = 0;
    }
    else
    {
      buffer[pos] = data;
      buffer[pos+1] = 0;
      pos += 1;
    }
  }  
}

void loop() {
  int c = command();
  Serial.print("command");
  Serial.println(c);
}
