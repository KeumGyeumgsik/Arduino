int cds = A1;
int LED = 12;

void setup()
{
  Serial.begin(9600);
  pinMode(cds, INPUT);
  pinMode(LED, OUTPUT);
}

void loop()
{
  cds = analogRead(A1);
  Serial.println(cds);
  if(cds<450)
  { 
    digitalWrite(LED1,LOW);
  }
  else
  {
    digitalWrite(LED1,HIGH);
  }
  delay(200);
}
