#ifndef __FARDUINO_UTILITIES__
#define __FARDUINO_UTILITIES__

#ifdef RASPBERRYPI_PICO
char *dtostrf(double val, signed char width, unsigned char prec, char *sout)
{
  //Commented code is the original version
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
#endif

unsigned long base_seconds = 0;
unsigned long base_fraction = 0;
unsigned long last_micros = 0;


unsigned long get_timestamp(){

  unsigned long current_micros = micros();
  if (current_micros<last_micros){
    base_seconds += 4294;
    base_fraction += 967295;
  }
  last_micros = current_micros;

  return current_micros;
}


void remove_spaces(char* my_buffer){

  char* my_pointer;

    while (*my_buffer != 0){
    if (*my_buffer==0x20){
      my_pointer = my_buffer;
      while (*my_pointer!=0){
        *my_pointer = *(my_pointer+1);
        my_pointer++;
      }
    }
    else{
      my_buffer++;
    }
  }

}

//writes time of day into given buffer, always 11 chars long
void time_of_day(unsigned long timestamp, char *destination) {

  int current_hour;
  int current_minute;
  int current_second;

  unsigned long timestamp_seconds = timestamp/1000000;
  unsigned long timestamp_fraction = timestamp%1000000;

  unsigned long current_fraction = base_fraction + timestamp_fraction;
  current_second = base_seconds + timestamp_seconds + current_fraction/1000000;
  current_fraction = (current_fraction%1000000)/100;


  current_hour =  current_second / 3600;
  current_second = current_second % 3600;
  current_minute = current_second / 60;
  current_second %= 60;

  sprintf(&destination[0], "%02i", current_hour);
  sprintf(&destination[2], "%02i", current_minute);
  sprintf(&destination[4], "%02i", current_second);
  destination[6] = '.';
  sprintf(&destination[7], "%04i", current_fraction);
  destination[11] = 0;

}



void construct_IMU_sentence (unsigned long timestamp, double my_acc[3], double my_gyro[3], double my_magn[3], char* sentence_buffer) {

  char *buffer_start;                          //pointer to timestamp string

  unsigned char xor_checksum;                 //simple XOR checksum
  int k, l;
  char *checksum_pointer;                            //pointer for checksum calculation

  buffer_start = sentence_buffer+8;            //remember beginning of sentence after first word
  checksum_pointer = sentence_buffer + 1;            //set pointer to first character after '$' for checksum calculation at the end of sentence construction
  sprintf(sentence_buffer, "$RQIMU0,");        //write sentence keyword to buffer
  sentence_buffer += 8;                        //adjust pointer to next free entry
  time_of_day(timestamp, sentence_buffer);        //construct string from timestamp
  sentence_buffer += 11;                       //adjust pointer to next free entry

  *sentence_buffer++ = ',';                    //add separator and adjust pointer

  for (int i = 0; i < 3; i++) {                //loop through acceleration vector
    dtostrf(my_acc[i], 6, 2, sentence_buffer);
    sentence_buffer += 6;
    *sentence_buffer++ = ',';
  }

  for (int i = 0; i < 3; i++) {                //loop through angular rate vector
    dtostrf(my_gyro[i], 6, 2, sentence_buffer);
    sentence_buffer += 6;
    *sentence_buffer++ = ',';
  }

  for (int i = 0; i < 2; i++) {                //loop through magnetic field vector
    dtostrf(my_magn[i], 6, 2, sentence_buffer);
    sentence_buffer += 6;
    *sentence_buffer++ = ',';
  }

  dtostrf(my_magn[2], 6, 2, sentence_buffer);
  sentence_buffer += 6;
  *sentence_buffer++ = '*';

  remove_spaces(buffer_start);

  xor_checksum = 0;                            //calculate checksum
  while (*checksum_pointer != '*') {
    xor_checksum ^= *checksum_pointer++;
  }
  sentence_buffer = checksum_pointer;
                                               //and append it to the sentence in hex format
  sprintf(sentence_buffer, "*%02X", xor_checksum);
  sentence_buffer += 3;
  *sentence_buffer++ = 0x0d;                  //NMEA sentence termination: $0D $0A
  *sentence_buffer++ = 0x0a;
  *sentence_buffer = 0;                       //terminate string
}




void construct_MET_sentence(unsigned long timestamp, double p, double T, double h, char* sentence_buffer) {

  char *buffer_start;                          //pointer to timestamp string
  char *time_string;
  unsigned char xor_checksum;
  int k, l;
  char *checksum_pointer;


  buffer_start = sentence_buffer;
  checksum_pointer = sentence_buffer + 1;
  sprintf(sentence_buffer, "$RQMET0,");
  sentence_buffer += 8;
  time_of_day(timestamp, sentence_buffer);
  sentence_buffer += 11;
  *sentence_buffer++ = ',';
  dtostrf(p, 7, 3, sentence_buffer);
  sentence_buffer += 7;
  *sentence_buffer++ = ',';

  dtostrf(T, 5, 2, sentence_buffer);
  sentence_buffer += 5;
  *sentence_buffer++ = ',';

  dtostrf(h, 7, 2, sentence_buffer);
  sentence_buffer += 6;

  *sentence_buffer++ = '*';
  remove_spaces(buffer_start);

  xor_checksum = 0;
  while (*checksum_pointer != '*') {
    xor_checksum ^= *checksum_pointer++;
  }
  sentence_buffer = checksum_pointer;

  sprintf(sentence_buffer, "*%02X", xor_checksum);
  sentence_buffer += 3;
  *sentence_buffer++ = 0x0d;
  *sentence_buffer++ = 0x0a;
  *sentence_buffer = 0;  //terminate string
}


void construct_state_sentence(unsigned long timestamp, double pressure_0, double pressure, stage_state_t my_state, char* sentence_buffer) {

  char *time_string;
  unsigned char xor_checksum;
  int k, l;
  char *checksum_pointer;

  checksum_pointer = sentence_buffer + 1;
  sprintf(sentence_buffer, "$RQSTATE,");
  sentence_buffer += 9;
  time_of_day(timestamp, sentence_buffer);
  sentence_buffer += 11;

  *sentence_buffer++ = ',';

  switch (my_state) {

    case state_IDLE: {
        sprintf(sentence_buffer, "IDLE,");
        sentence_buffer += 5;
        dtostrf(pressure_0, 7, 3, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_ACCELERATION: {
        sprintf(sentence_buffer, "ACCELERATION,");
        sentence_buffer += 13;
        dtostrf(pressure_0, 7, 3, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_LAUNCH: {
        sprintf(sentence_buffer, "LAUNCH,");
        sentence_buffer += 7;
        dtostrf(pressure_0, 7, 3, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_BURNOUT: {
        sprintf(sentence_buffer, "BURNOUT,");
        sentence_buffer += 8;
        dtostrf(pressure_0, 7, 3, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_SEPARATION: {
        sprintf(sentence_buffer, "SEPARATION,");
        sentence_buffer += 11;
        dtostrf(pressure_0, 7, 3, sentence_buffer);
        sentence_buffer += 7;
        break;
      }


    case state_COASTING: {
        sprintf(sentence_buffer, "COASTING,");
        sentence_buffer += 9;
        dtostrf(pressure, 7, 2, sentence_buffer);
        sentence_buffer += 7;
        *sentence_buffer++ = ',';
        dtostrf(pressure_0, 7, 2, sentence_buffer);
        sentence_buffer += 7;
        break;
      }

    case state_PEAK_REACHED: {
        sprintf(sentence_buffer, "PEAK_REACHED");
        sentence_buffer += 12;
        break;
	  }

    case state_FALLING: {
        sprintf(sentence_buffer, "FALLING");
        sentence_buffer += 7;
        break;
      }


    case state_DROGUE_OPENED: {
        sprintf(sentence_buffer, "DROGUE_OPENED");
        sentence_buffer += 13;
        break;
      }

    case state_LANDED: {
        sprintf(sentence_buffer, "LANDED");
        sentence_buffer += 6;
        break;
      }
  }

  xor_checksum = 0;
  while (checksum_pointer != sentence_buffer) {
    xor_checksum ^= *checksum_pointer++;
  }

  sprintf(sentence_buffer, "*%02X", xor_checksum);
  sentence_buffer += 3;
  *sentence_buffer++ = 0x0d;
  *sentence_buffer++ = 0x0a;
  *sentence_buffer = 0;  //terminate string
}


#endif
