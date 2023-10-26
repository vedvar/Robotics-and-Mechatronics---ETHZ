/// Lab 02 - Analog Signal Acquisition
// Sample the analog output of the hall sensor sensor
// and print the value to the serial port

// motor functions

void move_motor(int dist_mm)
{
  if (!calibrated)
  {
    calibrate_motor();
  }

  if (dir == forward)
  {
    digitalWrite(dirPin, forward);
  }
  else
  {
    digitalWrite(dirPin, LOW);
  }

  digitalWrite(sleep_pin, LOW); // make motor wake
  for (int x = 0; x < (stepsPerRevolution * (dist_mm / pitch)); x++)
  {
    if (!switch_press)
    {
      if ((dir == forward) && (position_x > 85.0))
      {
        // Serial.println("Limit reached, please reverse direction!");
        break;
      }
      else
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
        position_x = position_x + (2.0 * (float(dir) - 0.5)) * (float(pitch) / float(stepsPerRevolution)); // update position
      }
    }
    else
    {
      step_back();
      break;
    }
  }
  digitalWrite(sleep_pin, HIGH); // make motor sleep
  // Serial.println(position_x);
}

void switch_motor_direction()
{
  if (dir == forward)
  {
    dir = backward;
  }
  else
  {
    dir = forward;
  }
  // Serial.println(position_x);
}

void calibrate_motor()
{
  // set direction to toward motor/ board
  dir = backward;
  digitalWrite(dirPin, LOW);
  digitalWrite(sleep_pin, LOW); // make motor wake
  while (!switch_press)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  position_x = pos_offset;

  // set direction to away from motor/ board
  dir = forward;
  digitalWrite(dirPin, forward);

  step_back();
  calibrated = true;
  delay(10);
}

void step_back()
{
  digitalWrite(dirPin, forward); // direction away from motor
  digitalWrite(sleep_pin, LOW);  // make motor wake
  position_x = pos_offset;
  dir = forward;

  for (int x = 0; x < (stepsPerRevolution / pitch); x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    position_x = position_x + float(dir) * (float(pitch) / float(stepsPerRevolution));
  }
  delayMicroseconds(1000);
}

void button_ISR()
{ // Interrupts and system debouncing

  if ((millis() - last_pressed) > debounceTime)
  {
    switch_press = true;
    // Serial.println("Switch pressed!");
    last_pressed = millis();
  }
}
