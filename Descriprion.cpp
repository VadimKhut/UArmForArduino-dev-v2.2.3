

/*
Left Servo - Related with Arduino PWM PIN D13, Analog PIN 0 - (Lower_ArmLink2)
Right Servo Related with Arduino PWM PIN D12, Analog PIN 1 - (Upper_ArmLink1)
These two servos are used to control front ArmLink1 and back ArmLink2.
Hand Servo - Related with Arduino PWM PIN D10, Analog PIN 3 This mini-servo is responsible for
the rotation of end-effector of uArm, for example, Suction cup.

Z     X
|    /
|   /
|  /
| /
|/________________ Y




EXTERNAL_EEPROM_SYS_ADDRESS     0xA2
EXTERNAL_EEPROM_USER_ADDRESS    0xA0


EXTERNAL_EEPROM_SYS***
-----------------------------------------
mMaxAdcPos:
  offset = (4 + k) * 1024 + 500
  Adr= 4596; 5620; 6644; 7668


-----------------------------------------
unsigned char data[6]; // 0: L; 1: R; 2: Rotation; 3: hand rotation; 4:gripper; 5:pump











*/

#define NORMAL_MODE                 0
#define NORMAL_BT_CONNECTED_MODE    1
#define LEARNING_MODE               2
#define SINGLE_PLAY_MODE            3
#define LOOP_PLAY_MODE              4
#define LEARNING_MODE_STOP          5



















setup(){
	Init(){
		uArmInit()	{ // uArmAPI.cpp
			initHardware(){ // uArmAPI.cpp
				pinMode(BTN_D4, INPUT_PULLUP);
				pinMode(VALVE_EN, OUTPUT);
				buttonMenu.setPin(BTN_D4);
				buttonPlay.setPin(BTN_D7);
				buzzer.setPin(BUZZER);
			}
			controller.init(){ // uArmController.cpp
				servoOffset = readServoAngleOffset(i);  // i < 4
				if (isnan(servoOffset))
					EEPROM.put(MANUAL_OFFSET_ADDRESS + i * sizeof(servoOffset), servoOffset); //MANUAL_OFFSET_ADDRESS=30
				mServoAngleOffset[i] = servoOffset;
				for (int k = 0; k < 3; k++){
			        delay(10);
					unsigned char data[2];
					unsigned int offset = (4 + k) * 1024 + 500;
			        iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, offset, 2);
					mMaxAdcPos[k] = (data[0] << 8) + data[1];
				}
				mServo[SERVO_ROT_NUM].setPulseWidthRange(500, 2500);
				mServo[SERVO_LEFT_NUM].setPulseWidthRange(500, 2500);
				mServo[SERVO_RIGHT_NUM].setPulseWidthRange(500, 2500);
				mServo[SERVO_HAND_ROT_NUM].setPulseWidthRange(600, 2400);

				attachAllServo();  

				mCurAngle[0] = readServoAngle(SERVO_ROT_NUM, true);
				mCurAngle[1] = readServoAngle(SERVO_LEFT_NUM, true);
				mCurAngle[2] = readServoAngle(SERVO_RIGHT_NUM, true);
				mCurAngle[3] = readServoAngle(SERVO_HAND_ROT_NUM, true);
			}

			mCurStep = -1;
			mTotalSteps = -1; 	
		}
		service.init();	// empty

		serialCmdInit();

		moveTo(0, 150, 150); // do interpolation speed=100 (uArmAPI.cpp)

		Serial.println("@1");	// report ready
	}


}




loop(){


	run(){ // Don't remove

		handleSerialCmd();

		manage_inactivity(){ // Don't remove

			getSerialCmd();	// for serial communication

			// uArmService.cpp
			service.run(){	// for led, button, bt etc.

				systemRun(){

					//check the button4 status [NORMAL_MODE -> LEARNING_MODE -> LEARNING_MODE_STOP]
				   if (buttonMenu.clicked()){
						buttonMenu.clearEvent();
						switch (mSysStatus){

							case NORMAL_MODE:
							case NORMAL_BT_CONNECTED_MODE:
								mSysStatus = LEARNING_MODE;
								mRecordAddr = 0;//recording/playing address
								controller.detachAllServo();
								break;

							case LEARNING_MODE:
								//LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
								mSysStatus = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly
                
								pumpOff();
             
								break;

							default: break;
						}
					}

					//check the button7 status			// [NORMAL_MODE -> LOOP_PLAY_MODE]
					if (buttonPlay.longPressed()){

						switch(mSysStatus){
							case NORMAL_MODE:
								mRecordAddr = 0;
								mSysStatus = LOOP_PLAY_MODE;
								break;
						}
					}
					else if (buttonPlay.clicked()){		// [NORMAL_MODE -> SINGLE_PLAY_MODE]
					   switch(mSysStatus){
							case NORMAL_MODE:
								mRecordAddr = 0;//recording/playing address
								mSysStatus = SINGLE_PLAY_MODE;  // or play just one time
								break;

							case SINGLE_PLAY_MODE:
							case LOOP_PLAY_MODE:
								pumpOff();
								mSysStatus = NORMAL_MODE;
								break;

							case LEARNING_MODE:				// if LEARNING_MODE [pumpOff() -> pumpOn()]
								if (getPumpStatus())
								{
									pumpOff();
								}
								else
								{
									pumpOn();
								}    
								break;
						}

					}

				}

				if (millis() - mTickRecorderTime >= 50){
					mTickRecorderTime= millis();
					recorderTick(){

						switch(mSysStatus){		//every 0.05s=50ms
						
							case SINGLE_PLAY_MODE:
								
								if(play() == false){
									mSysStatus = NORMAL_MODE;
									mRecordAddr = 0;
								}
								break;

							case LOOP_PLAY_MODE:

								if(play() == false){
									mRecordAddr = 0;
								}
								break;

							case LEARNING_MODE:
							case LEARNING_MODE_STOP:

								if(record() == false){
									mSysStatus = NORMAL_MODE;
									mRecordAddr = 0;
           
									controller.attachAllServo();

								}
								break;

							default: 
								break;
						}

					}
									
				}
			}
			if(millis() - tickStartTime >= TICK_INTERVAL){		// TICK_INTERVAL=50ms
				tickStartTime = millis();
				tickTaskRun(){
					tickTimeOut();	// empty

					buttonPlay.tick(); // mState = HALF_PRESSED; RELEASE; LONGPRESSED ...
					buttonMenu.tick();
				}
			}   

		}

	}

}





bool uArmService::play(){

    unsigned char data[5];	// 0: Left; 1: Right; 2: Rotation; 3: Hand rotation; 4: Pump

    recorder.read(mRecordAddr, data, 5);
	debugPrint("mRecordAddr = %d, data=%d, %d, %d", mRecordAddr, data[0], data[1], data[2]);

    if(data[0] != 255){

        //double x, y, z;
        //controller.getXYZFromAngle(x, y, z, (double)data[2], (double)data[0], (double)data[1]);
        //moveToAngle((double)data[2], (double)data[0], (double)data[1]);
    	controller.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
        controller.writeServoAngle(SERVO_HAND_ROT_NUM, (double)data[3]);
        unsigned char pumpStatus = getPumpStatus() > 0 ? 1 : 0;
        if (pumpStatus != data[4]){

            if (data[4]){

                pumpOn();
            }
            else{

                pumpOff();
            }   
        }
    }
    else{

        pumpOff();
         
        return false;
    }

    mRecordAddr += 5;

    return true;
}




// uArmcontroller.cpp
writeServoAngle(byte servoNum, double servoAngle, boolean writeWithOffset){

  servoAngle = constrain(servoAngle, 0.00, 180.00);
  mCurAngle[servoNum] = servoAngle;
  servoAngle = writeWithOffset ? (servoAngle + mServoAngleOffset[servoNum]) : servoAngle;
  mServo[servoNum].write(servoAngle, mServoSpeed);
}






bool uArmService::record(){
	
	debugPrint("mRecordAddr = %d", mRecordAddr);

    if(mRecordAddr <= 65530){

        unsigned char data[5];	// 0: Left; 1: Right; 2: Rotation; 3: Hand rotation; 4: Pump

        if((mRecordAddr != 65530) && (mSysStatus != LEARNING_MODE_STOP)){

    		double rot, left, right;
    		//controller.updateAllServoAngle();
            controller.readServoAngles(rot, left, right);
			data[0] = (unsigned char)left;
			data[1] = (unsigned char)right;
            data[2] = (unsigned char)rot;
            data[3] = (unsigned char)controller.readServoAngle(SERVO_HAND_ROT_NUM);
            data[4] = getPumpStatus() > 0 ? 1 : 0;

            debugPrint("l=%d, r=%d, r= %d", data[0], data[1], data[2]);
        }
        else{
            data[0] = 255;	//255 is the ending flag
            recorder.write(mRecordAddr, data, 5);

            return false;
        }

        recorder.write(mRecordAddr, data, 5);
        mRecordAddr += 5;

        return true;
    }
    else{
        return false;
    }

}



readServoAngle(byte servoNum, boolean withOffset ){
  angle = analogToAngle(servoNum, getServoAnalogData(servoNum));
}


getServoAnalogData(byte servoNum){
  return getAnalogPinValue(SERVO_ANALOG_PIN[servoNum]);
}

analogToAngle(byte servoNum, int inputAnalog){
  readLinearOffset(servoNum, intercept, slope){
    EEPROM.get(LINEAR_INTERCEPT_START_ADDRESS + servoNum * sizeof(interceptVal), interceptVal);  // (70 + servoNum) * 4); EEPROM[70, 71, 72, 73]  
    EEPROM.get(LINEAR_SLOPE_START_ADDRESS + servoNum * sizeof(slopeVal), slopeVal);              // (50 + servoNum) * 4); EEPROM[50, 51, 52, 53]
  }
  angle = intercept + slope * inputAnalog;
}