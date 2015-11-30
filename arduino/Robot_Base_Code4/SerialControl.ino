void set_left_value(String the_string){
  if(the_string.substring(0,1) == "L"){
    char temp[20];
    the_string.substring(1).toCharArray(temp, 19);
    int l_val = atoi(temp);
    left = l_val;
  }
}

void set_right_value(String the_string){
  if(the_string.substring(0,1) == "R"){
    char temp[20];
    the_string.substring(1).toCharArray(temp, 19);
    int r_val = atoi(temp);
    right = r_val;
  }
}


void handle_command(String readString){

  set_left_value(readString);
  set_right_value(readString);


  // Here you can send the values back to your Computer and read them on the Processing terminal.
  // Sending these values over Xbee can take slow the sketch down, so I comment them out after testing. 
  
   Serial.print("left: ");
   Serial.print(left);
   Serial.print("     ");
   Serial.print("right: ");
   Serial.print(right);
   Serial.println("     ");
    

} 
  
