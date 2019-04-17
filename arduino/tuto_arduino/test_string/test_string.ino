void setup() {
  // put your setup code here, to run once:
    // Open serial port
    Serial.begin(9600);

    Serial.print("hello");

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  //int x = random(256);
  //Serial.println(x);

  if (Serial.available())
  {
    char* input = Serial.read();
    Serial.println(input);
    //char input[] ="123|456|789|444";
    char sep[] = "|";
 
    int count = 0;
    char* tab[10] = { 0 };
 
    char* token = strtok(input, sep);
 
    while ( token != NULL ) {
      if (count < 10) {
          int val = atoi(token);
          Serial.print(token);
          Serial.print(" val = ");
          Serial.println(val);
      } else {
          break;
      }
      token = strtok (NULL, sep);
    }
  }
}
