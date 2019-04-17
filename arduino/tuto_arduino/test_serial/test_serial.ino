
long speed = 9600;
//long speed = 115200;


void setup() {
  Serial.begin(speed); // use the same baud-rate as the python side
}

char buffer[256];
int pos = 0;

long number = 0;

void loop() {
  if(Serial.available() > 0) {
    char data = Serial.read();
    if (data == '#')
    {
      number += 1;
      Serial.print('(');
      Serial.print(number);
      Serial.print(')');
      Serial.print(buffer);

      char delims[] = "|=";
      char* ptr = strtok(buffer, delims);

      while(ptr != NULL) {
          // create next part
          int value = atoi(ptr);
          Serial.print(" - ");
          Serial.print(value);
          ptr = strtok(NULL, delims);
      }

      Serial.println("");

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
